#include <stddef.h>
#include <stdio.h>
#include <mujoco/mujoco.h>
int main(void) {
  printf("{\"m_nuser_jnt\":%zu,\"m_nbody\":%zu,\"m_body_plugin\":%zu,"
         "\"m_body_jntnum\":%zu,\"m_body_jntadr\":%zu,\"m_body_dofnum\":%zu,\"m_body_dofadr\":%zu,"
         "\"m_jnt_stiffness\":%zu,\"m_jnt_user\":%zu,\"m_jnt_limited\":%zu,"
         "\"m_jnt_qposadr\":%zu,\"m_qpos_spring\":%zu,\"m_jnt_range\":%zu,"
         "\"d_plugin_data\":%zu,\"d_qpos\":%zu,\"d_qfrc_passive\":%zu}\n",
         offsetof(mjModel,nuser_jnt),offsetof(mjModel,nbody),offsetof(mjModel,body_plugin),
         offsetof(mjModel,body_jntnum),offsetof(mjModel,body_jntadr),offsetof(mjModel,body_dofnum),offsetof(mjModel,body_dofadr),
         offsetof(mjModel,jnt_stiffness),offsetof(mjModel,jnt_user),offsetof(mjModel,jnt_limited),
         offsetof(mjModel,jnt_qposadr),offsetof(mjModel,qpos_spring),offsetof(mjModel,jnt_range),
         offsetof(mjData,plugin_data),offsetof(mjData,qpos),offsetof(mjData,qfrc_passive));
  return 0;
}
