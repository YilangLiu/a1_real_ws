;; Auto-generated. Do not edit!


(when (boundp 'mujoco_ros_msgs::SensorState)
  (if (not (find-package "MUJOCO_ROS_MSGS"))
    (make-package "MUJOCO_ROS_MSGS"))
  (shadow 'SensorState (find-package "MUJOCO_ROS_MSGS")))
(unless (find-package "MUJOCO_ROS_MSGS::SENSORSTATE")
  (make-package "MUJOCO_ROS_MSGS::SENSORSTATE"))

(in-package "ROS")
;;//! \htmlinclude SensorState.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mujoco_ros_msgs::SensorState
  :super ros::object
  :slots (_header _sensor ))

(defmethod mujoco_ros_msgs::SensorState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:sensor __sensor) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _sensor __sensor)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:sensor
   (&rest __sensor)
   (if (keywordp (car __sensor))
       (send* _sensor __sensor)
     (progn
       (if __sensor (setq _sensor (car __sensor)))
       _sensor)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; mujoco_ros_msgs/SensorBase[] _sensor
    (apply #'+ (send-all _sensor :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; mujoco_ros_msgs/SensorBase[] _sensor
     (write-long (length _sensor) s)
     (dolist (elem _sensor)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; mujoco_ros_msgs/SensorBase[] _sensor
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _sensor (let (r) (dotimes (i n) (push (instance mujoco_ros_msgs::SensorBase :init) r)) r))
     (dolist (elem- _sensor)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get mujoco_ros_msgs::SensorState :md5sum-) "c6a3e7ef3cb5a31ce0b40d3f6aa28a2f")
(setf (get mujoco_ros_msgs::SensorState :datatype-) "mujoco_ros_msgs/SensorState")
(setf (get mujoco_ros_msgs::SensorState :definition-)
      "Header header


SensorBase[] sensor

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: mujoco_ros_msgs/SensorBase
string name

float64[] data

")



(provide :mujoco_ros_msgs/SensorState "c6a3e7ef3cb5a31ce0b40d3f6aa28a2f")


