# Modified from JetBot Motor module
#
import atexit
import traitlets
from traitlets.config.configurable import Configurable
from jetbot import create

class Motor(Configurable):

    value = traitlets.Float()
    
    # config
    alpha = traitlets.Float(default_value=1.0).tag(config=True)
    beta = traitlets.Float(default_value=0.0).tag(config=True)

    channel_id = None

    def __init__(self, driver, channel, *args, **kwargs):
        super(Motor, self).__init__(*args, **kwargs)  # initializes traitlets

        self._motor = driver
        self.channel_id = channel
        atexit.register(self._release)
        
    @traitlets.observe('value')
    def _observe_value(self, change):
        self._write_value(change['new'])

    def _write_value(self, value):
        if(value > 1.0):
            value = 1.0
        if(value < -1.0):
            value = -1.0
       
        #"""Sets motor value between [-1, 1]"""
        #mapped_value = int(255.0 * (self.alpha * value + self.beta))
        #speed = min(max(abs(mapped_value), 0), 255)

		# Presuming value is a ratio between -1 to 1, and max 
		# speed of roomba is 50 cm / sec.
        speed = value * 50

        if( self.channel_id == 1):
            self._motor.setLeftWheelVel(speed)
        if( self.channel_id == 2):
            self._motor.setRightWheelVel(speed)

    def _release(self):
        """Stops motor by releasing control"""
        self._motor.stop()
        # Possibly consider closing serial port
        pass
