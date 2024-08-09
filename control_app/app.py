from flask import Flask, request, jsonify, render_template
import rospy
from std_msgs.msg import Int32, Float32, String, Bool

app = Flask(__name__)

# Initialize ROS node
rospy.init_node('flask_ros_publisher', anonymous=True)
button_pub = rospy.Publisher('button_topic', String, queue_size=10)
temperature_pub = rospy.Publisher('temperature', Float32, queue_size=10)
battery_pub = rospy.Publisher('battery_level', Float32, queue_size=10)
gps_accuracy_pub = rospy.Publisher('gps_accuracy', Float32, queue_size=10)
signal_strength_pub = rospy.Publisher('signal_strength', Float32, queue_size=10)
mode_pub = rospy.Publisher('mode_topic', String, queue_size=10)
stop_pub = rospy.Publisher('emergency_stop', Bool, queue_size=10)

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/button', methods=['POST'])
def button():
    data = request.json
    button_id = data.get('button_id')
    mode = data.get('mode')
    # Publish button ID and mode to ROS topic
    rospy.loginfo(f"Button {button_id} pressed in {mode} mode")

    if (button_id == "emergency_stop"):
        stop_pub.publish(True)
    if (button_id == "start"):
        stop_pub.publish(False)


    button_pub.publish(f"{button_id}")



    # Return a JSON response
    return jsonify({"message": f"Button {button_id} in {mode} mode response received"})

@app.route('/data', methods=['POST'])
def data():
    data = request.json
    temperature = data.get('temperature')
    battery = data.get('battery')
    gps_accuracy = data.get('gps_accuracy')
    signal_strength = data.get('signal_strength')
    
    # Publish the data to respective ROS topics
    rospy.loginfo(f"Temperature: {temperature}")
    temperature_pub.publish(float(temperature))
    
    rospy.loginfo(f"Battery Level: {battery}")
    battery_pub.publish(float(battery))
    
    rospy.loginfo(f"GPS Accuracy: {gps_accuracy}")
    gps_accuracy_pub.publish(float(gps_accuracy))
    
    rospy.loginfo(f"Signal Strength: {signal_strength}")
    signal_strength_pub.publish(float(signal_strength))
    
    # Return a JSON response
    return jsonify({"message": "Data received and published to ROS topics"})

@app.route('/mode', methods=['POST'])
def mode():
    data = request.json
    mode = data.get('mode')
    # Publish mode to ROS topic
    rospy.loginfo(f"Mode changed to {mode}")
    mode_pub.publish(mode)
    # Return a JSON response
    return jsonify({"message": f"Mode changed to {mode}"})

if __name__ == '__main__':
    app.run(debug=True)
