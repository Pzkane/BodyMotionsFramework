import math
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from std_msgs.msg import String

from bm_framework_ros2_pkg.ros_driver.parser import LimbData, LimbDataParser, TorsoData, TorsoDataParser


class BodyMotionsServerNode(Node):
    def __init__(self, *args, **kwargs):
        super(BodyMotionsServerNode, self).__init__(node_name="bm_server", namespace="body_motions_framework", *args, **kwargs)
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.__torso_data = TorsoData(None, None, None)
        self.__init_relay_connection()
        self.__init_boards_connrections()

    def __init_boards_connrections(self):
        self.create_subscription(String, "/body_motions_framework/limb_readings", self.__cb_limb_readings, 1)
        self.get_logger().info("Limb sensors connected!")

    def __init_relay_connection(self):
        self.create_subscription(String, "/body_motions_framework/mmr_relay", self.__cb_relay_data, 1)
        self.get_logger().info("Relay connected!")


    def __cb_relay_data(self, meta_motion_r_data: String):
        self.__torso_data = TorsoDataParser.deserialize(meta_motion_r_data.data)

    def __cb_limb_readings(self, nodemcu_data: String):
        try:
            self.__limbs_data: LimbData = LimbDataParser.deserialize(nodemcu_data.data)
        except AttributeError:
            # Skip packet
            return
        self.get_logger().info(f"Torso: {str(self.__torso_data)}, limbs: {str(self.__limbs_data)}")
        """
        rPitch: float = math.round((token.getPitchR()-8) * 100.0) / 100.0);
        lPitch: float = new Float(Math.round((token.getPitchL()-8) * 100.0) / 100.0);
                gloveTextToDash = new Float(-1 * Math.round((token.getRollL()+10) * 100.0) / 100.0).toString() + "\t\t" + token.getRollR().toString()
                        + "\n" + lPitch.toString() + "\t\t" + rPitch.toString();

                try {
                    // Main magic for low hands detection
                    _sensor_T3.getBoard();
                    // Threshold based on data set and has linear relationship between hand pitch and T3 Y values
                    final Float threshold = 5.f;
                    Float functionR = new Float(Math.round( ((25.7143 - 0.714286 * rPitch)) * 100.0) / 100.0);
                    Float functionL = new Float(Math.round( ((25.7143 - 0.714286 * lPitch)) * 100.0) / 100.0);

                    gloveTextToDash += "\n" + new Float(-1 * Math.round((_sensor_T3.getQuaternion().w() * 100) * 100.0) / 100.0).toString() + "\t\t"
                            + new Float(-1 * Math.round((_sensor_T3.getQuaternion().x() * 100) * 100.0) / 100.0).toString() + "\t\t"
                            + new Float(-1 * Math.round((_sensor_T3.getQuaternion().y() * 100) * 100.0) / 100.0).toString() + "\t\t"
                            + new Float(-1 * Math.round((_sensor_T3.getQuaternion().z() * 100) * 100.0) / 100.0).toString()
                            + "\n" + functionR.toString();
                    // By experimenting values ranges between -20 and {threshold}
                    // AND is beyond time threshold
                    final Integer downDelay = 1500;
                    if (functionR >= threshold || functionL >= threshold) {
                        if (!_handsDownRegistered) {
                            _handsDownRegistered = true;
                            _lastHandsDownTime = System.currentTimeMillis();
                        }
                        if (System.currentTimeMillis() - _lastHandsDownTime >= downDelay) {
                            gloveTextToDash = "LOW HANDS\n" + functionL.toString() + ":" + functionR.toString();
                        }
                    } else {
                        _handsDownRegistered = false;
                    }

                } catch (Exception e) {
                    // Just proceed if board is not loaded yet...
                }

                // Register punches
                final Integer delay = 300; // delay in ms between punches
                final Integer punchBaseline = 4000; // Tweak this for database access
                final Integer accelLeft = Math.abs(token.get_xL() + token.get_yL() + token.get_zL());
                final Integer accelRight = Math.abs(token.get_xR() + token.get_yR() + token.get_zR());
                Integer accelLeftPrev = DashboardFragment.lastAccelLeftToken != null
                        ? Math.abs(DashboardFragment.lastAccelLeftToken.get_xL()
                            + DashboardFragment.lastAccelLeftToken.get_yL()
                            + DashboardFragment.lastAccelLeftToken.get_zL())
                        : 0;
                Integer accelRightPrev = DashboardFragment.lastAccelRightToken != null
                        ? Math.abs(DashboardFragment.lastAccelRightToken.get_xR()
                            + DashboardFragment.lastAccelRightToken.get_yR()
                            + DashboardFragment.lastAccelRightToken.get_zR())
                        : 0;

                // Get time again for gloves after so many ticks
                Long timestamp = System.currentTimeMillis();

                if (accelLeft >= punchBaseline && accelLeft > accelLeftPrev) {
                    DashboardFragment.lastAccelLeftToken = token;
                } else if (accelLeft < accelLeftPrev && accelLeftPrev >= punchBaseline) {
                    if (_timeframeL == null || System.currentTimeMillis() - _timeframeL >= delay) {
                        Glove gl = new Glove();
                        gl.time = System.currentTimeMillis();
                        gl.glove = 'L';
                        gl.x = DashboardFragment.lastAccelLeftToken.get_xL();
                        gl.y = DashboardFragment.lastAccelLeftToken.get_yL();
                        gl.z = DashboardFragment.lastAccelLeftToken.get_zL();
                        gl.roll = DashboardFragment.lastAccelLeftToken.getRollL();
                        gl.pitch = DashboardFragment.lastAccelLeftToken.getPitchL();
                        registerAndInsertT3(gl.time);
                        insertAndTransfer(gl, EntityType.Glove);
                        _timeframeL = timestamp;
                        System.out.println("Hit Left! " + _hitCountL++);
                        DashboardFragment.flashLGlove(MainActivity.this, getResources().getColor(R.color.flashL, getTheme()));
                    }
                    DashboardFragment.lastAccelLeftToken = token;
                }

                if (accelRight >= punchBaseline && accelRight > accelRightPrev) {
                    DashboardFragment.lastAccelRightToken = token;
                } else if (accelRight < accelRightPrev && accelRightPrev >= punchBaseline) {
                    if (_timeframeR == null || System.currentTimeMillis() - _timeframeR >= delay) {
                        Glove gl = new Glove();
                        gl.time = System.currentTimeMillis();
                        gl.glove = 'R';
                        gl.x = DashboardFragment.lastAccelRightToken.get_xR();
                        gl.y = DashboardFragment.lastAccelRightToken.get_yR();
                        gl.z = DashboardFragment.lastAccelRightToken.get_zR();
                        gl.roll = DashboardFragment.lastAccelRightToken.getRollR();
                        gl.pitch = DashboardFragment.lastAccelRightToken.getPitchR();
                        registerAndInsertT3(gl.time);
                        insertAndTransfer(gl, EntityType.Glove);
                        _timeframeR = timestamp;
                        System.out.println("Hit Right!" + _hitCountR++);
                        DashboardFragment.flashRGlove(MainActivity.this, getResources().getColor(R.color.flashR, getTheme()));
                    }
                    DashboardFragment.lastAccelRightToken = token;
                }

                if (DashboardFragment.seriesL != null)
                    DashboardFragment.seriesL.appendData(new DataPoint(new Date(),accelLeft), true, 60);
                if (DashboardFragment.seriesR != null)
                    DashboardFragment.seriesR.appendData(new DataPoint(new Date(),accelRight), true, 60);

//                gloveTextToDash += text;
                if (homeText != null)
                    homeText.setText(String.format("%s", gloveTextToDash));
                if (dashGraphL != null) {
                    dashGraphL.onDataChanged(false, true);
                    dashGraphR.onDataChanged(false, true);
                }
    """
