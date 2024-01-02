; Auto-generated. Do not edit!


(cl:in-package a1_msgs-msg)


;//! \htmlinclude MultiFootPlanDiscrete.msg.html

(cl:defclass <MultiFootPlanDiscrete> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (feet
    :reader feet
    :initarg :feet
    :type (cl:vector a1_msgs-msg:FootPlanDiscrete)
   :initform (cl:make-array 0 :element-type 'a1_msgs-msg:FootPlanDiscrete :initial-element (cl:make-instance 'a1_msgs-msg:FootPlanDiscrete))))
)

(cl:defclass MultiFootPlanDiscrete (<MultiFootPlanDiscrete>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MultiFootPlanDiscrete>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MultiFootPlanDiscrete)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_msgs-msg:<MultiFootPlanDiscrete> is deprecated: use a1_msgs-msg:MultiFootPlanDiscrete instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <MultiFootPlanDiscrete>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:header-val is deprecated.  Use a1_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'feet-val :lambda-list '(m))
(cl:defmethod feet-val ((m <MultiFootPlanDiscrete>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:feet-val is deprecated.  Use a1_msgs-msg:feet instead.")
  (feet m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MultiFootPlanDiscrete>) ostream)
  "Serializes a message object of type '<MultiFootPlanDiscrete>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'feet))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'feet))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MultiFootPlanDiscrete>) istream)
  "Deserializes a message object of type '<MultiFootPlanDiscrete>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'feet) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'feet)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'a1_msgs-msg:FootPlanDiscrete))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MultiFootPlanDiscrete>)))
  "Returns string type for a message object of type '<MultiFootPlanDiscrete>"
  "a1_msgs/MultiFootPlanDiscrete")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MultiFootPlanDiscrete)))
  "Returns string type for a message object of type 'MultiFootPlanDiscrete"
  "a1_msgs/MultiFootPlanDiscrete")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MultiFootPlanDiscrete>)))
  "Returns md5sum for a message object of type '<MultiFootPlanDiscrete>"
  "24cfed7044607c942541950144570913")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MultiFootPlanDiscrete)))
  "Returns md5sum for a message object of type 'MultiFootPlanDiscrete"
  "24cfed7044607c942541950144570913")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MultiFootPlanDiscrete>)))
  "Returns full string definition for message of type '<MultiFootPlanDiscrete>"
  (cl:format cl:nil "# This is a message to hold a discrete foot plan for multiple robot feet~%#~%# The plans of each foot are stored in a vector of FootPlanDiscrete messages~%# (0 = front left, 1 = back left, 2 = front right, 3 = back right).~%# Accurate timing information to localize the plans is stored in the header~%~%Header header~%a1_msgs/FootPlanDiscrete[] feet~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: a1_msgs/FootPlanDiscrete~%# This is a message to hold the discrete foot plan for a single robot foot~%#~%# The plan is defined as a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%a1_msgs/FootState[] footholds~%~%================================================================================~%MSG: a1_msgs/FootState~%# This is a message to hold the state of a single foot of a legged robot~%#~%# The states of each foot are stored in a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%uint32 traj_index~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%bool contact~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MultiFootPlanDiscrete)))
  "Returns full string definition for message of type 'MultiFootPlanDiscrete"
  (cl:format cl:nil "# This is a message to hold a discrete foot plan for multiple robot feet~%#~%# The plans of each foot are stored in a vector of FootPlanDiscrete messages~%# (0 = front left, 1 = back left, 2 = front right, 3 = back right).~%# Accurate timing information to localize the plans is stored in the header~%~%Header header~%a1_msgs/FootPlanDiscrete[] feet~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: a1_msgs/FootPlanDiscrete~%# This is a message to hold the discrete foot plan for a single robot foot~%#~%# The plan is defined as a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%a1_msgs/FootState[] footholds~%~%================================================================================~%MSG: a1_msgs/FootState~%# This is a message to hold the state of a single foot of a legged robot~%#~%# The states of each foot are stored in a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%uint32 traj_index~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%bool contact~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MultiFootPlanDiscrete>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'feet) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MultiFootPlanDiscrete>))
  "Converts a ROS message object to a list"
  (cl:list 'MultiFootPlanDiscrete
    (cl:cons ':header (header msg))
    (cl:cons ':feet (feet msg))
))
