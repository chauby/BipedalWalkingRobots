#-*-coding:UTF-8 -*-
import sys
sys.path.append('./VREP_remoteAPIs')
import sim as vrep_sim

# LIPM simulation model for VREP
class LIPMSimModel():
    def __init__(self, name='LIPM_model'):
        """
        :param: name: string
            name of objective
        """
        super(self.__class__, self).__init__()
        self.name = name
        self.client_ID = None

        self.COM_handle = None
        self.left_hip_joint_handle = None
        self.left_knee_joint_handle = None
        self.right_hip_joint_handle = None
        self.right_knee_joint_handle = None
        self.left_foot_force_sensor_handle = None
        self.right_foot_force_sensor_handle = None

        self.COM_position = [0 ,0, 0] # position with x, y, z
        self.COM_linear_velocity = [0 ,0, 0] # linear velocity with x, y, z
        self.COM_angular_velocity = [0 ,0, 0] # linear velocity with x, y, z

        # joint angles
        self.left_hip_angle = 0
        self.left_knee_angle = 0
        self.right_hip_angle = 0
        self.right_knee_angle = 0

    def initializeSimModel(self, client_ID):
        try:
            print ('Connected to remote API server')
            client_ID != -1
        except:
            print ('Failed connecting to remote API server')

        self.client_ID = client_ID

        return_code, self.COM_handle = vrep_sim.simxGetObjectHandle(client_ID, 'COM', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object COM handle ok.')

        return_code, self.left_hip_joint_handle = vrep_sim.simxGetObjectHandle(client_ID, 'left_hip', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object left hip joint handle ok.')

        return_code, self.left_knee_joint_handle = vrep_sim.simxGetObjectHandle(client_ID, 'left_knee', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object left knee joint handle ok.')

        return_code, self.right_hip_joint_handle = vrep_sim.simxGetObjectHandle(client_ID, 'right_hip', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object right hip joint handle ok.')

        return_code, self.right_knee_joint_handle = vrep_sim.simxGetObjectHandle(client_ID, 'right_knee', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object right knee joint handle ok.')

        return_code, self.right_knee_joint_handle = vrep_sim.simxGetObjectHandle(client_ID, 'right_knee', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object right knee joint handle ok.')

        return_code, self.left_foot_force_sensor_handle = vrep_sim.simxGetObjectHandle(client_ID, 'left_foot_force_sensor', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object left foot force sensor handle ok.')

        return_code, self.right_foot_force_sensor_handle = vrep_sim.simxGetObjectHandle(client_ID, 'right_foot_force_sensor', vrep_sim.simx_opmode_blocking)
        if (return_code == vrep_sim.simx_return_ok):
            print('get object right foot force sensor handle ok.')

        return_code, pos = vrep_sim.simxGetObjectPosition(self.client_ID, self.COM_handle, -1, vrep_sim.simx_opmode_streaming)
        return_code, linear_vel, angular_vel = vrep_sim.simxGetObjectVelocity(self.client_ID, self.COM_handle, vrep_sim.simx_opmode_streaming)
        
        return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.left_hip_joint_handle, vrep_sim.simx_opmode_streaming)
        return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.left_knee_joint_handle, vrep_sim.simx_opmode_streaming)
        return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.right_hip_joint_handle, vrep_sim.simx_opmode_streaming)
        return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.right_knee_joint_handle, vrep_sim.simx_opmode_streaming)

        return_code, state, foot_pressure, torque = vrep_sim.simxReadForceSensor(self.client_ID, self.left_foot_force_sensor_handle, vrep_sim.simx_opmode_streaming)
        return_code, state, foot_pressure, torque = vrep_sim.simxReadForceSensor(self.client_ID, self.right_foot_force_sensor_handle, vrep_sim.simx_opmode_streaming)


        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.left_hip_joint_handle, 0, vrep_sim.simx_opmode_streaming)
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.left_knee_joint_handle, 0, vrep_sim.simx_opmode_streaming)
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.right_hip_joint_handle, 0, vrep_sim.simx_opmode_streaming)
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.right_knee_joint_handle, 0, vrep_sim.simx_opmode_streaming)
    
    def getJointAngle(self, joint_name):
        """
        :param: vrep_sim: int
            the vrep_sim handle
        :param: client_ID: int
            the client ID
        :param: joint_name: str
            the joint name: can be left_hip, left_knee, right_hip, right_knee
        """
        q = 0
        if joint_name == 'left_hip':
            return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.left_hip_joint_handle, vrep_sim.simx_opmode_buffer)
        elif joint_name == 'left_knee':
            return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.left_knee_joint_handle, vrep_sim.simx_opmode_buffer)
        elif joint_name == 'right_hip':
            return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.right_hip_joint_handle, vrep_sim.simx_opmode_buffer)
        elif joint_name == 'right_knee':
            return_code, q = vrep_sim.simxGetJointPosition(self.client_ID, self.right_knee_joint_handle, vrep_sim.simx_opmode_buffer)
        else:
            print('Error: joint name: \' ' + joint_name + '\' can not be recognized.')

        return q

    def getCOMPosition(self):
        return_code, pos = vrep_sim.simxGetObjectPosition(self.client_ID, self.COM_handle, -1, vrep_sim.simx_opmode_buffer)
        return pos

    def getCOMVelocity(self):
        return_code, linear_vel, angular_vel = vrep_sim.simxGetObjectVelocity(self.client_ID, self.COM_handle, vrep_sim.simx_opmode_buffer)
        return linear_vel, angular_vel

    def getFootPressure(self, foot_name):
        """
        :param: foot_name: str
            the foot's name, can be left_foot or right_foot
        """
        foot_pressure = 0

        if foot_name == 'left_foot':
            return_code, state, foot_pressure, torque = vrep_sim.simxReadForceSensor(self.client_ID, self.left_foot_force_sensor_handle, vrep_sim.simx_opmode_buffer)
        elif foot_name == 'right_foot':
            return_code, state, foot_pressure, torque = vrep_sim.simxReadForceSensor(self.client_ID, self.right_foot_force_sensor_handle, vrep_sim.simx_opmode_buffer)
        else:
            print('Error: foot name: \' ' + foot_name + '\' can not be recognized.')

        return foot_pressure

    def setJointAngle(self, q):
        """
        :param: q: float array of size 4 x 1
            the desired joint angle for all joints
        """
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.left_hip_joint_handle, q[0], vrep_sim.simx_opmode_streaming)
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.left_knee_joint_handle, q[1], vrep_sim.simx_opmode_streaming)
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.right_hip_joint_handle, q[2], vrep_sim.simx_opmode_streaming)
        vrep_sim.simxSetJointTargetPosition(self.client_ID, self.right_knee_joint_handle, q[3], vrep_sim.simx_opmode_streaming)
