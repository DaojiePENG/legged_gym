# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg, LeggedRobotCfgPPO
import math
class Go2RoughPropriCfg( LeggedRobotCfg ):
    class env(LeggedRobotCfg.env):
        num_envs = 4096 # 3072
        # num_actions = 12 
        num_observations = 48 # 改成48，仅仅保留本体感知proprioceptive，仍然在复杂环境下训练
    
    class terrain( LeggedRobotCfg.terrain ):
        # mesh_type = 'plane'
        measure_heights = False # False: 基础的 48 个输入信息； True： 额外添加 187 个高度感知信息，共 235 个点； 

    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.30] # x,y,z [m]
        p_scale = [1.0, 1.0, 1.0] # 设置初始位置的缩放
        default_joint_angles = { # = target angles [rad] when action = 0.0
            'FL_hip_joint': 0.015*p_scale[0],   # [rad]
            'RL_hip_joint': 0.01*p_scale[0],   # [rad]
            'FR_hip_joint': -0.015*p_scale[0] ,  # [rad]
            'RR_hip_joint': -0.01*p_scale[0],   # [rad]

            'FL_thigh_joint': 0.8*p_scale[1],     # [rad]
            'RL_thigh_joint': 0.8*p_scale[1],   # [rad]
            'FR_thigh_joint': 0.8*p_scale[1],     # [rad]
            'RR_thigh_joint': 0.8*p_scale[1],   # [rad]

            'FL_calf_joint': -1.6*p_scale[2],   # [rad]
            'RL_calf_joint': -1.6*p_scale[2],    # [rad]
            'FR_calf_joint': -1.6*p_scale[2],  # [rad]
            'RR_calf_joint': -1.6*p_scale[2],    # [rad]
        }

    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P'
        stiffness = {'joint': 20.}  # [N*m/rad]
        damping = {'joint': 0.5}     # [N*m*s/rad]
        # action scale: target angle = actionScale * action + defaultAngle
        action_scale = 0.25
        # decimation: Number of control action updates @ sim DT per policy DT
        decimation = 4

    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/go2/urdf/go2_description.urdf'
        name = "go2"
        foot_name = "foot"
        penalize_contacts_on = ["thigh", "calf"]
        terminate_after_contacts_on = ["base"]
        self_collisions = 1 # 1 to disable, 0 to enable...bitwise filter
  
    class rewards( LeggedRobotCfg.rewards ):
        soft_dof_pos_limit = 0.9
        base_height_target = 0.30
        max_contact_force = 300
        class scales( LeggedRobotCfg.rewards.scales ):
            torques = -0.0002
            dof_pos_limits = -10.0
            stand_still = -0.1
            # orientation = -0.5
            # base_height = -0.1
            dof_acc = -2.5e-7*2

class Go2RoughPropriCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.01
    class runner( LeggedRobotCfgPPO.runner ):
        run_name = ''
        max_iterations = 50000 # number of policy updates
        experiment_name = 'rough_propri_go2' # registry 的名称要和这里的一致。不一致也可以，train的调用主要是 registry 处定义的。

  