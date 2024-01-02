; Auto-generated. Do not edit!


(cl:in-package a1_msgs-msg)


;//! \htmlinclude FootPlanDiscrete.msg.html

(cl:defclass <FootPlanDiscrete> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (footholds
    :reader footholds
    :initarg :footholds
    :type (cl:vector a1_msgs-msg:FootState)
   :initform (cl:make-array 0 :element-type 'a1_msgs-msg:FootState :initial-element (cl:make-instance 'a1_msgs-msg:FootState))))
)

(cl:defclass FootPlanDiscrete (<FootPlanDiscrete>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <FootPlanDiscrete>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'FootPlanDiscrete)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name a1_msgs-msg:<FootPlanDiscrete> is deprecated: use a1_msgs-msg:FootPlanDiscrete instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <FootPlanDiscrete>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:header-val is deprecated.  Use a1_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'footholds-val :lambda-list '(m))
(cl:defmethod footholds-val ((m <FootPlanDiscrete>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader a1_msgs-msg:footholds-val is deprecated.  Use a1_msgs-msg:footholds instead.")
  (footholds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <FootPlanDiscrete>) ostream)
  "Serializes a message object of type '<FootPlanDiscrete>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'footholds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'footholds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <FootPlanDiscrete>) istream)
  "Deserializes a message object of type '<FootPlanDiscrete>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'footholds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'footholds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'a1_msgs-msg:FootState))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<FootPlanDiscrete>)))
  "Returns string type for a message object of type '<FootPlanDiscrete>"
  "a1_msgs/FootPlanDiscrete")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'FootPlanDiscrete)))
  "Returns string type for a message object of type 'FootPlanDiscrete"
  "a1_msgs/FootPlanDiscrete")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<FootPlanDiscrete>)))
  "Returns md5sum for a message object of type '<FootPlanDiscrete>"
  "2ae5c98d4a7907bd7964a0959d454c94")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'FootPlanDiscrete)))
  "Returns md5sum for a message object of type 'FootPlanDiscrete"
  "2ae5c98d4a7907bd7964a0959d454c94")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<FootPlanDiscrete>)))
  "Returns full string definition for message of type '<FootPlanDiscrete>"
  (cl:format cl:nil "# This is a message to hold the discrete foot plan for a single robot foot~%#~%# The plan is defined as a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%a1_msgs/FootState[] footholds~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: a1_msgs/FootState~%# This is a message to hold the state of a single foot of a legged robot~%#~%# The states of each foot are stored in a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%uint32 traj_index~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%bool contact~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'FootPlanDiscrete)))
  "Returns full string definition for message of type 'FootPlanDiscrete"
  (cl:format cl:nil "# This is a message to hold the discrete foot plan for a single robot foot~%#~%# The plan is defined as a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%a1_msgs/FootState[] footholds~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: a1_msgs/FootState~%# This is a message to hold the state of a single foot of a legged robot~%#~%# The states of each foot are stored in a vector of FootState messages~%# Accurate timing information is stored in the header~%~%Header header~%uint32 traj_index~%geometry_msgs/Vector3 position~%geometry_msgs/Vector3 velocity~%geometry_msgs/Vector3 acceleration~%bool contact~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <FootPlanDiscrete>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'footholds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <FootPlanDiscrete>))
  "Converts a ROS message object to a list"
  (cl:list 'FootPlanDiscrete
    (cl:cons ':header (header msg))
    (cl:cons ':footholds (footholds msg))
))
