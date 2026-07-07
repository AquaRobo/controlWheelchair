from utils.Configurator import Configurator
import time

class JoystickPostProcessors:
    """Singleton joystick input processor with debounced button detection.

    Normalizes raw sensor_msgs/Joy data into named buttons/axes (PS-style
    layout: x/o/tri/rect, L1/R1, D-pad from axes 6/7) and exposes semantic
    button names (e.g. MAP_SAVE, KITCHEN_SAVE) mapped from
    joystick_buttons.yaml as class constants. Shared by MapSaverNode and
    RoomPoseSaverNode — the singleton means one node's Joy callback can feed
    every consumer.

    Input:  updateData(buttons_data, axis_data) from a Joy message.
    Output: isPressed(name) — press count within a time window while held;
            isClicked(name) — rising-edge count reported once per window;
            getAxis() — dict of stick/trigger axis values.
    """

    _instance = None
    def __new__(cls, *args, **kwargs):
        if cls._instance is None:  # Ensure a single instance
            cls._instance = super(JoystickPostProcessors, cls).__new__(cls)
        return cls._instance  # Always return the same instance

    def __init__(self):
        if not hasattr(self, "_initialized"):  # Ensure __init__ runs only once
            self._initialized = True
            self.previous_button_states = {}
            self.button_press_timestamps = {}  # Store button press timestamps
            self.last_click_time = {}  # Store the last registered click time
            self.hold_counts = {}  # Track how long a button is held
            self.start_time = {}  # Store when the first click happened
            self.waiting_period = {}  # Flag to track waiting period
            self.click_counts = {}  # Track clicks before returning a count
            self.data_with_timestamp = {}
            self.__constructConstants()

    @classmethod
    def __constructConstants(cls):
        """
        Dynamically create class constants from the configuration file.
        This ensures that button mappings are automatically assigned as class attributes.

        Example Config:
        {
            "button_0": "FLASH",
            "button_x": "LEFTGRIPPER_OPEN",
            "button_l1": "LEFTGRIPPER_CLOSE",
            "button_4": "RIGHTGRIPPER_OPEN"
        }
        """
        joystickButtons = Configurator().fetchData(Configurator.BUTTONS)
        if joystickButtons:
            for button_key, button_name in joystickButtons.items():
                if button_key.startswith("button"):  # Ensure valid button key format
                    try:
                        button_number = button_key.replace("button", "")
                        # print("ana ahooo: ")
                        # print(button_number)
                        setattr(cls, button_name, button_number)
                        setattr(cls, f"_{button_number}", button_number)  # Add support for _X button notation
                    except ValueError:
                        print(f"Invalid button number format: '{button_key}'")
                        continue  # Skip invalid keys


    def __constructJoystickData(self, buttons_data, axis_data):
        buttons_dict = {
            "button_x": bool(buttons_data[0]),
            "button_o": bool(buttons_data[1]),
            "button_tri": bool(buttons_data[2]),
            "button_rect": bool(buttons_data[3]),
            "button_l1": bool(buttons_data[4]),
            "button_r1": bool(buttons_data[5]),
            "button_l2": bool(buttons_data[6]),
            "button_r2": bool(buttons_data[7]),
            "button_share": bool(buttons_data[8]),
            "button_options": bool(buttons_data[9]),
            "button_ps": bool(buttons_data[10]),
            "button_l3": bool(buttons_data[11]),
            "button_r3": bool(buttons_data[12]),
            "button_right": axis_data[6] < 0,
            "button_top": axis_data[7] > 0,
            "button_left": axis_data[6] > 0,
            "button_bot": axis_data[7] < 0,
        }
        axes_dict = {
            "left_x_axis": axis_data[0],
            "left_y_axis": axis_data[1],
            "trigger_left_axis": axis_data[2],
            "right_x_axis": axis_data[3],
            "right_y_axis": axis_data[4],
            "trigger_right_axis": axis_data[5],
        }
        return buttons_dict, axes_dict
    
    def updateData(self, buttons_data, axis_data):
        """
        Write the joystick data (buttons and axes).
        
        Parameters:
            buttons_data (object): Button data (can be any serializable object like a dictionary).
            axis_data (list): List containing joystick axis values [left_x, left_y, right_x, right_y].
        """
        try:
            buttons, axes = self.__constructJoystickData(buttons_data, axis_data)
            self.data_with_timestamp = {
                "timestamp": time.time(),
                "buttons": buttons,
                "axes": axes,
            }
        except Exception as e:
            print(f"Failed to update data: {e}")

    def isPressed(self, button_name, time_window=0.4):
        """
        Count the number of times a button was pressed within the last `time_window` seconds.
        The count remains as long as the button is being held and also detects consecutive presses.
        If the last press is still held, it does not reset to zero. The function keeps returning 0 until
        the time window expires, then returns the count. If the button is released, the count resets to zero.

        Parameters:
            button_name (str): The name of the button constant (e.g., LEFTGRIPPER_OPEN, _1).
            time_window (float): The time window in seconds to count presses.

        Returns:
            int: The number of presses detected within the specified time window.

        Raises:
            ValueError: If the button name is not defined.
        """
        data = self.data_with_timestamp
        if data is None or "buttons" not in data:
            return 0  # No data available

        # Resolve button number (either "_X" or BUTTON_NAME)
        if button_name.startswith("_"):
            try:
                button_number = int(button_name[1:])  # Extract the number
            except ValueError:
                print(f"Invalid button number format: '{button_name}'")
                raise ValueError(f"Invalid button number format: '{button_name}'")
        else:
            button_number = getattr(self, button_name, None)

        if button_number is None:
            print(f"Button name '{button_name}' is not defined.")
            raise ValueError(f"Button name '{button_name}' is not defined.")

        button_key = f"button{button_number}"

        # Ensure the button exists in received data
        if button_key not in data["buttons"]:
            print(f"Button key '{button_key}' not found in data.")
            return 0

        current_state = data["buttons"].get(button_key, False)
        current_time = time.time()

        # Initialize tracking structures if not present
        if button_key not in self.previous_button_states:
            self.previous_button_states[button_key] = False
            self.button_press_timestamps[button_key] = []
            self.last_click_time[button_key] = 0
            self.hold_counts[button_key] = 0
            self.start_time[button_key] = 0
            self.waiting_period[button_key] = True

        previous_state = self.previous_button_states[button_key]

        # Detect state change: False → True (rising edge)
        if current_state and not previous_state:
            self.button_press_timestamps[button_key].append(current_time)
            self.last_click_time[button_key] = current_time  # Store the last click time
            self.hold_counts[button_key] += 1  # Increase count when pressed
            if self.start_time[button_key] == 0:
                self.start_time[button_key] = current_time  # Start the waiting period
                self.waiting_period[button_key] = True

        # Update previous state
        self.previous_button_states[button_key] = current_state

        # If the waiting period is still active, return 0
        if self.waiting_period[button_key] and (current_time - self.start_time[button_key] < time_window):
            return 0

        # End the waiting period once the time window expires
        self.waiting_period[button_key] = False

        # If the button is released, reset to 0
        if not current_state:
            self.hold_counts[button_key] = 0
            self.start_time[button_key] = 0
            self.waiting_period[button_key] = True
            return 0

        return self.hold_counts[button_key]

    
    def isClicked(self, button_name, time_window=0.4):
        """
        Count the number of times a button was clicked (state changed from False to True)
        within the last `time_window` seconds. The function keeps returning 0 until
        the time window expires, then returns the count, regardless of whether the button is held.

        Parameters:
            button_name (str): The name of the button constant (e.g., LEFTGRIPPER_OPEN, _1).
            time_window (float): The time window in seconds to count clicks.

        Returns:
            int: The number of state changes (False → True) detected within the specified time window.

        Raises:
            ValueError: If the button name is not defined.
        """
        data = self.data_with_timestamp
        if data is None or "buttons" not in data:
            return 0  # No data available

        # Resolve button number (either "_X" or BUTTON_NAME)
        if button_name.startswith("_"):
            try:
                button_number = int(button_name[1:])  # Extract the number
            except ValueError:
                print(f"Invalid button number format: '{button_name}'")
                raise ValueError(f"Invalid button number format: '{button_name}'")
        else:
            button_number = getattr(self, button_name, None)

        if button_number is None:
            print(f"Button name '{button_name}' is not defined.")
            raise ValueError(f"Button name '{button_name}' is not defined.")

        button_key = f"button{button_number}"

        # Ensure the button exists in received data
        if button_key not in data["buttons"]:
            print(f"Button key '{button_key}' not found in data.")
            return 0

        current_state = data["buttons"].get(button_key, False)
        current_time = time.time()

        # Initialize tracking structures if not present
        if button_key not in self.previous_button_states:
            self.previous_button_states[button_key] = False
            self.button_press_timestamps[button_key] = []
            self.last_click_time[button_key] = 0
            self.click_counts[button_key] = 0
            self.start_time[button_key] = 0
            self.waiting_period[button_key] = True

        previous_state = self.previous_button_states[button_key]

        # Detect state change: False → True (rising edge)
        if current_state and not previous_state:
            self.button_press_timestamps[button_key].append(current_time)
            self.last_click_time[button_key] = current_time  # Store the last click time
            self.click_counts[button_key] += 1  # Increase count when clicked
            if self.start_time[button_key] == 0:
                self.start_time[button_key] = current_time  # Start the waiting period
                self.waiting_period[button_key] = True

        # Update previous state
        self.previous_button_states[button_key] = current_state

        # If the waiting period is still active, keep storing the count but return 0
        if self.waiting_period[button_key] and (current_time - self.start_time[button_key] < time_window):
            return 0

        # End the waiting period once the time window expires
        self.waiting_period[button_key] = False

        # Store the final count to return exactly once
        final_count = self.click_counts[button_key]
        
        # Reset all tracking variables for the next detection cycle
        self.click_counts[button_key] = 0
        self.start_time[button_key] = 0
        self.waiting_period[button_key] = True

        return final_count
    
    
    def getAxis(self):
        """
        Returns an array of joystick axis values.
        
        Returns:
            dictionary: Dictionary of joystick axis values {left_x_axis: 0.0, left_y_axis: 0.0, right_x_axis: 0.0, right_y_axis: 0.0}.
        """
        data = self.data_with_timestamp
        if data is None or "axes" not in data:
            return {"left_x_axis": 0.0, "left_y_axis": 0.0, "right_x_axis": 0.0, "right_y_axis": 0.0, "trigger_left_axis": 0.0, "trigger_right_axis": 0.0}  # Default values if no data available
        return data["axes"]
