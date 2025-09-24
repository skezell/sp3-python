class Robot:
    name = "Robot"
    hardware_configuration = {}
    navigation = None
    sensors = None
    attachments = {}
    custom_configuration = {}

    def _set_cfg_value(self, attr, new_value):
        if new_value:
            setattr(self, attr, new_value)

    def __init__(self, name=None, hardware_configuration = {}, **kwargs):
        self.name = self._set_cfg_value("name", name)
        self.hardware_configuration = self._set_cfg_value("hardware_configuration", hardware_configuration)
        self.custom_configuration = self._set_cfg_value("custom_configuration", kwargs)
        

    def _setup_hardware(self):
        pass

    def _setup_navigation(self):
        pass
    
    def _setup_sensors(self):
        pass
    
    def _setup_attachments(self):
        pass
    


    