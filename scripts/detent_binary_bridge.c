/* Diagnostic bridge for ONE pinned, user-supplied x86-64 ELF detent plugin.
 * Does not dlopen ELF, execute its constructors, or implement its SDF plugins.
 * Calls the original init/compute/destroy instruction bytes with SysV ABI, after
 * validating all struct offsets those functions access. Four CRT calls are
 * adapted to Windows. Not a native Linux plugin or a general ELF loader.
 */
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <bcrypt.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <mujoco/mujoco.h>
#include "detent_bridge_config.h"

#if !defined(__x86_64__) || !defined(__GNUC__)
#error This diagnostic bridge requires x86-64 GCC sysv_abi support
#endif
#define OFFSET(type, field, value) _Static_assert(offsetof(type,field)==value,"ABI mismatch: " #field)
OFFSET(mjModel,nuser_jnt,0xf0); OFFSET(mjModel,nbody,0x10);
OFFSET(mjModel,body_plugin,0x618); OFFSET(mjModel,body_jntnum,0x578);
OFFSET(mjModel,body_jntadr,0x580); OFFSET(mjModel,body_dofnum,0x588);
OFFSET(mjModel,body_dofadr,0x590); OFFSET(mjModel,jnt_stiffness,0x6c0);
OFFSET(mjModel,jnt_user,0x6e0); OFFSET(mjModel,jnt_limited,0x688);
OFFSET(mjModel,jnt_qposadr,0x668); OFFSET(mjModel,qpos_spring,0x550);
OFFSET(mjModel,jnt_range,0x6c8); OFFSET(mjData,plugin_data,0x277f8);
OFFSET(mjData,qpos,0x27778); OFFSET(mjData,qfrc_passive,0x279b8);

#define SYSV __attribute__((sysv_abi))
static void* SYSV crt_malloc(size_t size) {return malloc(size);}
static void SYSV crt_free(void* pointer) {free(pointer);}
static int SYSV crt_puts(const char* value) {return puts(value);}
static double SYSV crt_round(double value) {return round(value);}

static unsigned char* image=NULL;
static const char* status="not_initialized";
__declspec(dllexport) const char* lab_detent_bridge_status(void) {return status;}

typedef struct {
  unsigned char ident[16]; uint16_t type,machine; uint32_t version;
  uint64_t entry,phoff,shoff; uint32_t flags;
  uint16_t ehsize,phentsize,phnum,shentsize,shnum,shstrndx;
} ElfHeader;
typedef struct {
  uint32_t type,flags; uint64_t offset,vaddr,paddr,filesz,memsz,align;
} ElfProgram;
_Static_assert(sizeof(ElfHeader)==64 && sizeof(ElfProgram)==56,"ELF layout mismatch");

static void trampoline(size_t offset,void* target) {
  /* movabs rax,target; jmp rax. These are this process's private PLT slots. */
  unsigned char code[12]={0x48,0xb8,0,0,0,0,0,0,0,0,0xff,0xe0};
  memcpy(code+2,&target,8); memcpy(image+offset,code,sizeof(code));
}

static int initialize_image(void) {
  FILE* file=fopen(DETENT_ELF_PATH,"rb");
  if (!file) {status="elf_read_failed";return 0;}
  fseek(file,0,SEEK_END);long length=ftell(file);rewind(file);
  if(length!=DETENT_ELF_SIZE) {fclose(file);status="elf_size_mismatch";return 0;}
  unsigned char* bytes=malloc((size_t)length);
  if(!bytes){fclose(file);status="allocation_failed";return 0;}
  size_t received=fread(bytes,1,(size_t)length,file);fclose(file);
  BCRYPT_ALG_HANDLE algorithm=NULL;BCRYPT_HASH_HANDLE hash=NULL;
  unsigned char digest[32];DWORD object_size=0,returned=0;unsigned char* object=NULL;
  NTSTATUS result=BCryptOpenAlgorithmProvider(&algorithm,BCRYPT_SHA256_ALGORITHM,NULL,0);
  if(result>=0)result=BCryptGetProperty(algorithm,BCRYPT_OBJECT_LENGTH,(PUCHAR)&object_size,sizeof(object_size),&returned,0);
  if(result>=0){object=malloc(object_size);if(!object)result=(NTSTATUS)0xC0000017;}
  if(result>=0)result=BCryptCreateHash(algorithm,&hash,object,object_size,NULL,0,0);
  if(result>=0)result=BCryptHashData(hash,bytes,(ULONG)length,0);
  if(result>=0)result=BCryptFinishHash(hash,digest,32,0);
  if(hash)BCryptDestroyHash(hash);
  free(object);
  if(algorithm) BCryptCloseAlgorithmProvider(algorithm,0);
  if(received!=(size_t)length || result<0 || memcmp(digest,DETENT_ELF_SHA256,32)) {
    free(bytes);status="elf_sha256_mismatch";return 0;
  }
  const ElfHeader* header=(const ElfHeader*)bytes;
  if(memcmp(header->ident,"\177ELF\2\1",6) || header->machine!=62 || header->phentsize!=56 ||
      header->phnum>16 || header->phoff+(uint64_t)header->phnum*56>(uint64_t)length) {
    free(bytes);status="elf_header_mismatch";return 0;
  }
  const size_t image_size=0x10000;
  image=VirtualAlloc(NULL,image_size,MEM_COMMIT|MEM_RESERVE,PAGE_READWRITE);
  if(!image){free(bytes);status="virtual_alloc_failed";return 0;}
  for(unsigned int i=0;i<header->phnum;i++) {
    const ElfProgram* p=(const ElfProgram*)(bytes+header->phoff+i*56);
    if(p->type!=1)continue;
    if(p->filesz>p->memsz || p->offset+p->filesz>(uint64_t)length || p->vaddr+p->memsz>image_size) {
      free(bytes);VirtualFree(image,0,MEM_RELEASE);image=NULL;status="elf_segment_mismatch";return 0;
    }
    memcpy(image+p->vaddr,bytes+p->offset,(size_t)p->filesz);
  }
  free(bytes);
  trampoline(0x1150,(void*)crt_free);trampoline(0x1160,(void*)crt_round);
  trampoline(0x1170,(void*)crt_puts);trampoline(0x1200,(void*)crt_malloc);
  DWORD previous;
  if(!VirtualProtect(image,image_size,PAGE_EXECUTE_READ,&previous)) {
    VirtualFree(image,0,MEM_RELEASE);image=NULL;status="virtual_protect_failed";return 0;
  }
  FlushInstructionCache(GetCurrentProcess(),image,image_size);
  status="pinned_original_detent_code_ready";
  return 1;
}

typedef int (SYSV *OriginalInit)(const mjModel*,mjData*,int);
typedef void (SYSV *OriginalDestroy)(mjData*,int);
typedef void (SYSV *OriginalCompute)(const mjModel*,mjData*,int,int);
static int init(const mjModel* m,mjData* d,int instance) {
  return ((OriginalInit)(image+0x1420))(m,d,instance);
}
static void destroy(mjData* d,int instance) {
  ((OriginalDestroy)(image+0x13e0))(d,instance);
}
static void compute(const mjModel* m,mjData* d,int instance,int capability) {
  ((OriginalCompute)(image+0x1850))(m,d,instance,capability);
}
static int nstate(const mjModel* m,int instance){(void)m;(void)instance;return 0;}
static void reset(const mjModel* m,mjtNum* state,void* data,int instance) {
  (void)m;(void)state;(void)data;(void)instance;
}

mjPLUGIN_LIB_INIT {
  if(!initialize_image()) return;
  mjpPlugin plugin;mjp_defaultPlugin(&plugin);
  plugin.name="mjlab.passive.detent";plugin.capabilityflags=mjPLUGIN_PASSIVE;
  plugin.nstate=nstate;plugin.init=init;plugin.destroy=destroy;
  plugin.reset=reset;plugin.compute=compute;
  if(mjp_registerPlugin(&plugin)<0)status="plugin_registration_failed";
}
