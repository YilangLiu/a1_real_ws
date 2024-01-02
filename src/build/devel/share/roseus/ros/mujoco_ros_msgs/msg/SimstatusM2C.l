;; Auto-generated. Do not edit!


(when (boundp 'mujoco_ros_msgs::SimstatusM2C)
  (if (not (find-package "MUJOCO_ROS_MSGS"))
    (make-package "MUJOCO_ROS_MSGS"))
  (shadow 'SimstatusM2C (find-package "MUJOCO_ROS_MSGS")))
(unless (find-package "MUJOCO_ROS_MSGS::SIMSTATUSM2C")
  (make-package "MUJOCO_ROS_MSGS::SIMSTATUSM2C"))

(in-package "ROS")
;;//! \htmlinclude SimstatusM2C.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass mujoco_ros_msgs::SimstatusM2C
  :super ros::object
  :slots (_header _time _sim_paused _sim_reset ))

(defmethod mujoco_ros_msgs::SimstatusM2C
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:time __time) 0.0)
    ((:sim_paused __sim_paused) nil)
    ((:sim_reset __sim_reset) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _time (float __time))
   (setq _sim_paused __sim_paused)
   (setq _sim_reset __sim_reset)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:time
   (&optional __time)
   (if __time (setq _time __time)) _time)
  (:sim_paused
   (&optional (__sim_paused :null))
   (if (not (eq __sim_paused :null)) (setq _sim_paused __sim_paused)) _sim_paused)
  (:sim_reset
   (&optional (__sim_reset :null))
   (if (not (eq __sim_reset :null)) (setq _sim_reset __sim_reset)) _sim_reset)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _time
    8
    ;; bool _sim_paused
    1
    ;; bool _sim_reset
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _time
       (sys::poke _time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _sim_paused
       (if _sim_paused (write-byte -1 s) (write-byte 0 s))
     ;; bool _sim_reset
       (if _sim_reset (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _time
     (setq _time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _sim_paused
     (setq _sim_paused (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _sim_reset
     (setq _sim_reset (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get mujoco_ros_msgs::SimstatusM2C :md5sum-) "59140af914df0ac28c7df06d452568bd")
(setf (get mujoco_ros_msgs::SimstatusM2C :datatype-) "mujoco_ros_msgs/SimstatusM2C")
(setf (get mujoco_ros_msgs::SimstatusM2C :definition-)
      "#MUJOCO -> CONTROLLER SIM STATUS COMMANDER


Header header
float64 time

bool sim_paused
bool sim_reset



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

")



(provide :mujoco_ros_msgs/SimstatusM2C "59140af914df0ac28c7df06d452568bd")


