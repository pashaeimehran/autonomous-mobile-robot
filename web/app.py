"""Browser-based manual control through ROS velocity commands."""

import os

from flask import Flask, abort, render_template
from geometry_msgs.msg import Twist
import rospy

app = Flask(__name__, template_folder="templates")
publisher = None

# Linear and angular demands for manual driving.
COMMANDS = {
    "forward": (0.12, 0.0),
    "backward": (-0.12, 0.0),
    "left": (0.0, 0.8),
    "right": (0.0, -0.8),
    "stop": (0.0, 0.0),
}


@app.route("/", methods=["GET"])
def index():
    return render_template("index.html")


@app.route("/api/drive/<command>", methods=["POST"])
def drive(command):
    if command not in COMMANDS:
        abort(404)
    linear, angular = COMMANDS[command]
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    publisher.publish(twist)
    return ("", 204)


if __name__ == "__main__":
    rospy.init_node("manual_web_control", disable_signals=True)
    publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    # Bind to loopback unless an interface is explicitly configured.
    app.run(host=os.environ.get("ROBOT_WEB_HOST", "127.0.0.1"),
            port=int(os.environ.get("ROBOT_WEB_PORT", "5000")),
            debug=False, use_reloader=False)
