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

from legged_gym import LEGGED_GYM_ROOT_DIR, envs
from time import time
from warnings import WarningMessage
import numpy as np
import os
import math

from isaacgym.torch_utils import *
from isaacgym import gymtorch, gymapi, gymutil

import torch
from torch import Tensor
from typing import Tuple, Dict

from legged_gym import LEGGED_GYM_ROOT_DIR
from legged_gym.envs.base.legged_robot import LeggedRobot
from legged_gym.utils.terrain_e import TerrainE
from legged_gym.utils.math import quat_apply_yaw, wrap_to_pi, torch_rand_sqrt_float
from legged_gym.utils.helpers import class_to_dict
from .legged_robot_config import LeggedRobotCfg

class LeggedRobotE(LeggedRobot):
    '''
    我想可以采用重写部分方法来实现目的；省的文件之间版本和注释太乱，不好管理；试试看吧，不行再重头 base_task 开始写；
    '''
    cfg : LeggedRobotCfg
    def __init__(self, cfg, sim_params, physics_engine, sim_device, headless):
        super().__init__(cfg, sim_params, physics_engine, sim_device, headless) # 复用 LeggedRobot 的初始化函数；

    def create_sim(self):
        """ Creates simulation, terrain and evironments (重写LeggedRobot的创建仿真函数，主要改变是采用自定义的地形图配置)

        该函数由 base_task 的初始化函数直接调用；
        """
        self.up_axis_idx = 2 # 2 for z, 1 for y -> adapt gravity accordingly
        self.sim = self.gym.create_sim(self.sim_device_id, self.graphics_device_id, self.physics_engine, self.sim_params)
        mesh_type = self.cfg.terrain.mesh_type
        if mesh_type in ['heightfield', 'trimesh']:
            self.terrain = TerrainE(self.cfg.terrain, self.num_envs) # 通过修改它来决定具体调用的地形类；
        if mesh_type=='plane':
            self._create_ground_plane()
        elif mesh_type=='heightfield':
            self._create_heightfield()
        elif mesh_type=='trimesh':
            self._create_trimesh()
        elif mesh_type is not None:
            raise ValueError("Terrain mesh type not recognised. Allowed types are [None, plane, heightfield, trimesh]")
        self._create_envs()


    def _resample_commands(self, env_ids):
        '''
        新特性：
        1. 基于所处地形环境的随机指令采样；

        新控制配置变量：
        1. selected_terrain_types 在 config 中定义的选择需要特殊处理的地形类型，用数字表示；

        该函数在 _post_physics_step_callback(self) 和 reset_idx(self, env_ids) 函数中被调用；
        '''
        """ Randommly select commands of some environments

        Args:
            env_ids (List[int]): Environments ids for which new commands are needed
        """
        self.commands[env_ids, 0] = torch_rand_float(self.command_ranges["lin_vel_x"][0], self.command_ranges["lin_vel_x"][1], (len(env_ids), 1), device=self.device).squeeze(1)
        self.commands[env_ids, 1] = torch_rand_float(self.command_ranges["lin_vel_y"][0], self.command_ranges["lin_vel_y"][1], (len(env_ids), 1), device=self.device).squeeze(1)
        if self.cfg.commands.heading_command:
            self.commands[env_ids, 3] = torch_rand_float(self.command_ranges["heading"][0], self.command_ranges["heading"][1], (len(env_ids), 1), device=self.device).squeeze(1)
        else:
            self.commands[env_ids, 2] = torch_rand_float(self.command_ranges["ang_vel_yaw"][0], self.command_ranges["ang_vel_yaw"][1], (len(env_ids), 1), device=self.device).squeeze(1)
        
        if self.cfg.commands.use_selected_terrain:
            '''基于地形的控制指令处理，启用基于指令的控制处理的时使用；
            如果是特定的地形，则不需要z方向旋转和y方向的速度命令，仅考虑x方向命令；
            '''
            selected_terrain_types = self.cfg.commands.selected_terrain_types # config 文件中定义的需特殊处理的地形；
            terrain_selected_ids  = torch.Tensor()
            for i in range(len(selected_terrain_types)):
                '''根据给定的标号，迭代地依次对相应的地形标号拼接到一起作为 terrain_selected_ids 它包含了所有选定地形的 env 的标号；'''
                selected_terrain_id = torch.nonzero(self.terrain_types==selected_terrain_types[i]) # 返回该地形类型标号对应的地形的标号（[0-num_envs]）；
                terrain_selected_ids = torch.cat([terrain_selected_ids, selected_terrain_id]) # 将多个地形选择结果组合；
            for env in env_ids:
                '''迭代寻找同时被输入的 env_ids 和 选定的地形 terrain_selected_ids 选中的元素；'''
                for terrain in terrain_selected_ids:
                    if env == terrain:
                        double_selected_ids = torch.cat([double_selected_ids, terrain])
            
            # 对于同时处于输入的 env_ids 和 选定的地形 terrain_selected_ids 的情况进行处理；
            # 根据 config 文件中 use_command_list 中的定义，决定禁止哪些指令；
            if not self.cfg.commands.use_command_list[0]:
                self.commands[double_selected_ids, 0] = torch.zeros_like(self.commands[double_selected_ids, 0]) # 取消y方向的速度控制命令；
            if not self.cfg.commands.use_command_list[1]:
                self.commands[double_selected_ids, 1] = torch.zeros_like(self.commands[double_selected_ids, 1]) # 取消y方向的速度控制命令；
            if not self.cfg.commands.use_command_list[2]:
                self.commands[double_selected_ids, 2] = torch.zeros_like(self.commands[double_selected_ids, 2]) # 取消z方向旋转的速度控制命令；
            if not self.cfg.commands.use_command_list[3]:
                self.commands[double_selected_ids, 3] = torch.zeros_like(self.commands[double_selected_ids, 3]) # 取消y方向的速度控制命令；
            # if not self.cfg.commands.use_command_list[0]:

        # set small commands to zero
        self.commands[env_ids, :2] *= (torch.norm(self.commands[env_ids, :2], dim=1) > 0.2).unsqueeze(1)

        
    def _reset_root_states(self, env_ids):
        '''
        新特性：
        1. 添加初始朝向随机化，为后面基于地形的控制命令方式做准备；
        '''
        """ Resets ROOT states position and velocities of selected environmments
            Sets base position based on the curriculum
            Selects randomized base velocities within -0.5:0.5 [m/s, rad/s]
        Args:
            env_ids (List[int]): Environemnt ids
        """
        # base position
        if self.custom_origins:
            '''
            这里实现了随机化的初始化位置，使得在一个 小terrain 中的 env 相互之间不是完全重合的；不过方向都是一样的；
            我可以试一下把初始化的方向也改成随机的，这样看起来就很乱了；
            '''
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
            self.root_states[env_ids, :2] += torch_rand_float(-1., 1., (len(env_ids), 2), device=self.device) # xy position within 1m of the center
            self.root_states[env_ids, 5:6] += torch_rand_float(-math.pi, math.pi, (len(env_ids),1),device=self.device) # z 方向在 0-360度 之间随机（头朝向随机，仅测试）；
        else:
            self.root_states[env_ids] = self.base_init_state
            self.root_states[env_ids, :3] += self.env_origins[env_ids]
        # base velocities
        self.root_states[env_ids, 7:13] = torch_rand_float(-0.5, 0.5, (len(env_ids), 6), device=self.device) # [7:10]: lin vel, [10:13]: ang vel
        env_ids_int32 = env_ids.to(dtype=torch.int32)
        self.gym.set_actor_root_state_tensor_indexed(self.sim,
                                                     gymtorch.unwrap_tensor(self.root_states),
                                                     gymtorch.unwrap_tensor(env_ids_int32), len(env_ids_int32))
        

    def _push_robots(self):
        '''
        新特性：
        1. 在z轴旋转方向上也给与随机推动导致的初速度；
        新控制配置变量：
        1. z轴旋转方向上也给与随机推动最大速度： max_push_vel_yaw ；
        '''
        """ Random pushes the robots. Emulates an impulse by setting a randomized base velocity. 
        """
        max_vel = self.cfg.domain_rand.max_push_vel_xy
        max_yaw = self.cfg.domain_rand.max_push_vel_yaw
        self.root_states[:, 7:9] = torch_rand_float(-max_vel, max_vel, (self.num_envs, 2), device=self.device) # lin vel x/y 
        self.root_states[:, 12:13] = torch_rand_float(-max_yaw, max_yaw, (self.num_envs, 1), device=self.device) # 在z轴旋转方向上也给与随机推动；
        # 也就是说从外部强制给它赋值一个初速度；但是要是本身 root_states 就不为 0 的情况呢？ 应该是在原有的速度上加上一个额外的推动吧？
        # 或者说这默认机器人是在静止状态被推动的？估计是。
        self.gym.set_actor_root_state_tensor(self.sim, gymtorch.unwrap_tensor(self.root_states))