from utils.Configurator import Configurator

class RoomPoseSaver:
    """Persists named room poses to room_poses.yaml.

    Thin persistence service used by RoomPoseSaverNode: keeps the room-pose
    dictionary in memory and writes it back through the Configurator whenever
    a pose is updated. AutoNavNode reads the same file to resolve navigation
    goals.

    Input:  room name plus position (x, y) and orientation quaternion via
            updatePose().
    Output: updated room_poses.yaml config file.
    """

    def __init__(self):
        self.configurator = Configurator("control")
        self.room_poses = self.configurator.fetchData(Configurator.ROOM_POSES)
    
    def updatePose(self, room_name:str, x_pose:float, y_pose:float, x_orientation:float, y_orientation:float, z_orientation:float, w_orientation:float) -> None:
        """Update the pose of a specified room in the configuration.

        Args:
            room_name (str): The name of the room to update.
            x_pose (float): The x position of the room.
            y_pose (float): The y position of the room.
            x_orientation (float): The x component of the room's orientation quaternion.
            y_orientation (float): The y component of the room's orientation quaternion.
            z_orientation (float): The z component of the room's orientation quaternion.
            w_orientation (float): The w component of the room's orientation quaternion.
        """
        self.room_poses[room_name] = {
            'position': {
                'x': x_pose,
                'y': y_pose
            },
            'orientation': {
                'x': x_orientation,
                'y': y_orientation,
                'z': z_orientation,
                'w': w_orientation
            }
        }
        self.configurator.setConfig(Configurator.ROOM_POSES, self.room_poses)