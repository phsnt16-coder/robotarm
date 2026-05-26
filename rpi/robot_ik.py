from ikpy.chain import Chain

class RobotIKController:
    # 하드웨어 및 기본 파라미터 고정 설정
    URDF_PATH = "mybot.urdf"
    ACTIVE_MASK = [False, True, True, True, False, True, False]
    DEFAULT_ORIENTATION = [0, 0, -1]
    ORIENTATION_MODE = "Z"

    # 로봇 모델 초기화
    def __init__(self):
        self.chain = Chain.from_urdf_file(
            self.URDF_PATH, 
            active_links_mask=self.ACTIVE_MASK
        )

    # 역기구학(IK) 계산 함수
    def calculate_ik(self, target_position):
        initial_joint_positions = [0] * len(self.chain.links)
        
        joints = self.chain.inverse_kinematics(
            target_position=target_position,
            target_orientation=self.DEFAULT_ORIENTATION,
            orientation_mode=self.ORIENTATION_MODE,
            initial_position=initial_joint_positions
        )
        return joints

    # 순기구학(FK) 검증 함수
    def verify_fk(self, joints):
        computed_matrix = self.chain.forward_kinematics(joints)
        return computed_matrix[:3, 3].tolist()