;; Auto-generated. Do not edit!


(when (boundp 'a1_msgs::FootPlanDiscrete)
  (if (not (find-package "A1_MSGS"))
    (make-package "A1_MSGS"))
  (shadow 'FootPlanDiscrete (find-package "A1_MSGS")))
(unless (find-package "A1_MSGS::FOOTPLANDISCRETE")
  (make-package "A1_MSGS::FOOTPLANDISCRETE"))

(in-package "ROS")
;;//! \htmlinclude FootPlanDiscrete.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass a1_msgs::FootPlanDiscrete
  :super ros::object
  :slots (_header _footholds ))

(defmethod a1_msgs::FootPlanDiscrete
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:footholds __footholds) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _footholds __footholds)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:footholds
   (&rest __footholds)
   (if (keywordp (car __footholds))
       (send* _footholds __footholds)
     (progn
       (if __footholds (setq _footholds (car __footholds)))
       _footholds)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; a1_msgs/FootState[] _footholds
    (apply #'+ (send-all _footholds :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; a1_msgs/FootState[] _footholds
     (write-long (length _footholds) s)
     (dolist (elem _footholds)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; a1_msgs/FootState[] _footholds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _footholds (let (r) (dotimes (i n) (push (instance a1_msgs::FootState :init) r)) r))
     (dolist (elem- _footholds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get a1_msgs::FootPlanDiscrete :md5sum-) "2ae5c98d4a7907bd7964a0959d454c94")
(setf (get a1_msgs::FootPlanDiscrete :datatype-) "a1_msgs/FootPlanDiscrete")
(setf (get a1_msgs::FootPlanDiscrete :definition-)
      "# This is a message to hold the discrete foot plan for a single robot foot
#
# The plan is defined as a vector of FootState messages
# Accurate timing information is stored in the header

Header header
a1_msgs/FootState[] footholds

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
MSG: a1_msgs/FootState
# This is a message to hold the state of a single foot of a legged robot
#
# The states of each foot are stored in a vector of FootState messages
# Accurate timing information is stored in the header

Header header
uint32 traj_index
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
bool contact

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
")



(provide :a1_msgs/FootPlanDiscrete "2ae5c98d4a7907bd7964a0959d454c94")


