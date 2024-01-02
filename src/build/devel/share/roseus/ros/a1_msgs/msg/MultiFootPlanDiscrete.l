;; Auto-generated. Do not edit!


(when (boundp 'a1_msgs::MultiFootPlanDiscrete)
  (if (not (find-package "A1_MSGS"))
    (make-package "A1_MSGS"))
  (shadow 'MultiFootPlanDiscrete (find-package "A1_MSGS")))
(unless (find-package "A1_MSGS::MULTIFOOTPLANDISCRETE")
  (make-package "A1_MSGS::MULTIFOOTPLANDISCRETE"))

(in-package "ROS")
;;//! \htmlinclude MultiFootPlanDiscrete.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass a1_msgs::MultiFootPlanDiscrete
  :super ros::object
  :slots (_header _feet ))

(defmethod a1_msgs::MultiFootPlanDiscrete
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:feet __feet) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _feet __feet)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:feet
   (&rest __feet)
   (if (keywordp (car __feet))
       (send* _feet __feet)
     (progn
       (if __feet (setq _feet (car __feet)))
       _feet)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; a1_msgs/FootPlanDiscrete[] _feet
    (apply #'+ (send-all _feet :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; a1_msgs/FootPlanDiscrete[] _feet
     (write-long (length _feet) s)
     (dolist (elem _feet)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; a1_msgs/FootPlanDiscrete[] _feet
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _feet (let (r) (dotimes (i n) (push (instance a1_msgs::FootPlanDiscrete :init) r)) r))
     (dolist (elem- _feet)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get a1_msgs::MultiFootPlanDiscrete :md5sum-) "24cfed7044607c942541950144570913")
(setf (get a1_msgs::MultiFootPlanDiscrete :datatype-) "a1_msgs/MultiFootPlanDiscrete")
(setf (get a1_msgs::MultiFootPlanDiscrete :definition-)
      "# This is a message to hold a discrete foot plan for multiple robot feet
#
# The plans of each foot are stored in a vector of FootPlanDiscrete messages
# (0 = front left, 1 = back left, 2 = front right, 3 = back right).
# Accurate timing information to localize the plans is stored in the header

Header header
a1_msgs/FootPlanDiscrete[] feet

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
MSG: a1_msgs/FootPlanDiscrete
# This is a message to hold the discrete foot plan for a single robot foot
#
# The plan is defined as a vector of FootState messages
# Accurate timing information is stored in the header

Header header
a1_msgs/FootState[] footholds

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



(provide :a1_msgs/MultiFootPlanDiscrete "24cfed7044607c942541950144570913")


