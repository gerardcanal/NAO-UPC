
from smach import StateMachine, Concurrence

from naokinator_ros.smach_AkinatorAnswer import GetUserAnswer
from nao_smach_utils.joint_trajectory_state import JointAngleState
from nao_smach_utils.random_selection_state import RandomSelectionFromPoolState


class SpeechRecognitionAndGesture(Concurrence):

    _movementList = [[0, 0], [0, 0], [0, 0], [-0.5, 0], [0, -0.5], [0.5, 0], [0, 0.5]]

    def getfinish_Cb(self, outcome_map):
        if outcome_map['GET_USER_ANSWER'] == 'succeeded':
            return True
        return False

    def outcome_Cb(self, outcome_map):
        if outcome_map['GET_USER_ANSWER'] == 'succeeded':
            return 'succeeded'
        return 'aborted'

    def __init__(self):
        Concurrence(default_outcome='aborted',
                    input_keys=['text'],
                    output_keys=['text'],
                    outcomes=['succeeded', 'preempted', 'aborted'],
                    child_termination_cb=self.getfinish_Cb,
                    outcome_cb=self.outcome_Cb)

        jointLoop = StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        with jointLoop:
            StateMachine.add('NEXT_MOVE',
                             RandomSelectionFromPoolState(self._movementList),
                             output_keys={'joint_angles'},
                             transitions={'succeeded': 'MOVEMENT'}
                             )
            StateMachine.add('MOVEMENT',
                             JointAngleState(['HeadPitch', 'HeadYaw']),
                             transitions={'succeeded': 'NEXT_MOVE'}
                             )

        with self:
            Concurrence.add('MOVING',
                            jointLoop,
                            transitions={'succeeded': 'succeeded'}
                            )

            Concurrence.add('GET_USER_ANSWER',
                            GetUserAnswer(),
                            transitions={'succeeded': 'succeeded',
                                         'aborted': 'aborted'},
                            remapping={'text': 'text'}
                            )
