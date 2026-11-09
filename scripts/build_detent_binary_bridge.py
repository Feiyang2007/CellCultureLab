"""Build a diagnostic Windows ABI bridge for a SHA-pinned original ELF detent.

No changes to source MJCF, source library, Isaac dependencies, or production USD.
The bridge executes original detent instruction bytes, not a re-guessed spring.
This is explicitly experimental: native Linux full-plugin acceptance is pending.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT=Path(__file__).resolve().parents[1]
PIN="9c855731e7e18277d9a03e4063e9d3cacd12072098cab4bff40464a630079320"


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--elf",type=Path,default=Path.home()/".codex/tmp-repos/AutoBio/autobio/libmjlab.so.3.3.0")
    parser.add_argument("--dependency-dir",type=Path,default=Path.home()/"AppData/LocalLow/CellCultureLabReference/mujoco330")
    parser.add_argument("--output-dir",type=Path,default=Path.home()/"AppData/LocalLow/CellCultureLabReference/detent_bridge")
    parser.add_argument("--gcc",type=Path,default=Path("C:/msys64/mingw64/bin/gcc.exe"))
    args=parser.parse_args()
    binary=args.elf.read_bytes()
    if hashlib.sha256(binary).hexdigest()!=PIN:
        raise RuntimeError("ELF differs from audited detent binary; do not use hard-coded offsets")
    output=args.output_dir.resolve();output.mkdir(parents=True,exist_ok=True)
    (output/"tmp").mkdir(exist_ok=True)
    header=("#define DETENT_ELF_PATH "+json.dumps(str(args.elf.resolve()))+"\n"
        +f"#define DETENT_ELF_SIZE {len(binary)}\n"
        +"static const unsigned char DETENT_ELF_SHA256[32]={"+','.join(str(x) for x in bytes.fromhex(PIN))+"};\n")
    (output/"detent_bridge_config.h").write_text(header,encoding="utf-8")
    environment=os.environ.copy()
    environment["TEMP"]=environment["TMP"]=str(output/"tmp")
    dependency=args.dependency_dir.resolve()/"mujoco"
    destination=output/"detent_binary_bridge.dll"
    source=ROOT/"scripts/detent_binary_bridge.c"
    command=[str(args.gcc),"-std=c11","-O2","-shared","-static-libgcc",str(source),
        "-I",str(dependency/"include"),"-I",str(output),str(dependency/"mujoco.dll"),
        "-lbcrypt","-o",str(destination)]
    subprocess.run(command,env=environment,check=True)
    report=dict(status="BUILT_NOT_RUNTIME_VALIDATED",bridge=str(destination),
        bridge_sha256=hashlib.sha256(destination.read_bytes()).hexdigest(),
        source=str(source),source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        original_elf=str(args.elf.resolve()),original_elf_sha256=PIN,
        execution_mode="original detent x86-64 instructions with Windows/SysV ABI and CRT bridge",
        original_elf_loaded_as_library=False,elf_constructors_executed=False,
        linux_native_full_plugin_validated=False,sdf_plugins_included=False,
        production_use_approved=False,compiler_command=command)
    (output/"build.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps(report),flush=True)


if __name__=="__main__":main()
