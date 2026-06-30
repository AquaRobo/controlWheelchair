import yaml 
from utils.UtilityMethods import UtilityMethods

class Configurator:
    SMOOTHING_STRAT = "smoothing_strategy"
    STEERING_STRAT = "steering_strategy"
    WHEELCHAIR_CONTROLLERS = "my_wheelchair_controller"
    MOTORS = "motors"
    COMM_HANDLER = "comm_config"
    PID_PARAMS = "pid_params"
    WHEELCHAIR_CONFIG = "wheelchair_config"
    BUTTONS = "joystick_buttons"
    ROOM_POSES = "room_poses"
    SPEECH_RECOGNIZER = "speech_recognizer"
    CAMERAS = "cameras"
    OBJECT_DETECTION = "object_detection"
    ROOM_IDENTIFIER = "room_identification"
    VOICE_NAVIGATION = "voice_navigation"

    def __init__(self, pkg_name: str = "control"):
        self.__config_file = ''
        self.pkg_name = pkg_name

    def __raiseTypeError(self, data_type: str):
        constants = [attr for attr in dir(self) if not callable(getattr(self, attr)) and not attr.startswith("_")] # Returns all constants of the class while rejecting those starting with '_'
        raise TypeError(f"Config file of type {data_type} doesn't exist, only {', '.join(constants)} are allowed.")

    def __getYamlFile(self, data_type: str) -> None:
        root = UtilityMethods.getPackageConfig(self.pkg_name)
        if data_type == Configurator.SMOOTHING_STRAT:
            self.__config_file = root + f"/{Configurator.SMOOTHING_STRAT}.yaml"
        elif data_type == Configurator.WHEELCHAIR_CONTROLLERS:
            self.__config_file = root + f"/{Configurator.WHEELCHAIR_CONTROLLERS}.yaml"
        elif data_type == Configurator.MOTORS:
            self.__config_file = root + f"/{Configurator.MOTORS}.yaml"
        elif data_type == Configurator.COMM_HANDLER:
            self.__config_file = root + f"/{Configurator.COMM_HANDLER}.yaml"
        elif data_type == Configurator.STEERING_STRAT:
            self.__config_file = root + f"/{Configurator.STEERING_STRAT}.yaml"
        elif data_type == Configurator.PID_PARAMS:
            self.__config_file = root + f"/{Configurator.PID_PARAMS}.yaml"
        elif data_type == Configurator.WHEELCHAIR_CONFIG:
            self.__config_file = root + f"/{Configurator.WHEELCHAIR_CONFIG}.yaml"
        elif data_type == Configurator.BUTTONS:
            self.__config_file = root + f"/{Configurator.BUTTONS}.yaml"
        elif data_type == Configurator.ROOM_POSES:
            self.__config_file = root + f"/{Configurator.ROOM_POSES}.yaml"
        elif data_type == Configurator.SPEECH_RECOGNIZER:
            self.__config_file = root + f"/{Configurator.SPEECH_RECOGNIZER}.yaml"
        elif data_type == Configurator.CAMERAS:
            self.__config_file = root + f"/{Configurator.CAMERAS}.yaml"
        elif data_type == Configurator.OBJECT_DETECTION:
            self.__config_file = root + f"/{Configurator.OBJECT_DETECTION}.yaml"
        elif data_type == Configurator.ROOM_IDENTIFIER:
            self.__config_file = root + f"/{Configurator.ROOM_IDENTIFIER}.yaml"
        elif data_type == Configurator.VOICE_NAVIGATION:
            self.__config_file = root + f"/{Configurator.VOICE_NAVIGATION}.yaml"
        else:
            self.__raiseTypeError(data_type)

    def fetchData(self, data_type: str) -> dict:
        try:
            self.__getYamlFile(data_type)
            with open(self.__config_file, 'r') as file:
                data = yaml.safe_load(file)
            return data if data is not None else {}
        except FileNotFoundError:
            print(f"Error: File '{self.__config_file}' not found.")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file '{self.__config_file}': {e}")
        except TypeError as e:
            print(e)

    def setConfig(self, data_type: str, new_data: dict) -> None:
        """
        Update the YAML configuration file with new_data.
        Only updates the keys provided in new_data and keeps other keys intact.

        :param data_type: Type of configuration (e.g., "smoothing_strategy")
        :param new_data: Dictionary containing the new key-value pairs to update.
        """

        try:
            self.__getYamlFile(data_type)
            try:
                with open(self.__config_file, 'r') as file:
                    existing_data = yaml.safe_load(file) or {}
            except FileNotFoundError:
                existing_data = {}
                print(f"Warning: {self.__config_file} not found. A new file will be created.")

            # Update existing data with new_data (merge dictionaries)
            updated_data = {**existing_data, **new_data}

            # Write back to file
            with open(self.__config_file, 'w') as file:
                yaml.safe_dump(updated_data, file, default_flow_style=False)

            print(f"Configuration for '{data_type}' updated successfully.")         

        except yaml.YAMLError as e:
            print(f"Error: Failed to write YAML data. Details: {e}")
        except TypeError as e:
            print(e)
