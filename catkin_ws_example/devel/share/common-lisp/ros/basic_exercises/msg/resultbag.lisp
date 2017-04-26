; Auto-generated. Do not edit!


(cl:in-package basic_exercises-msg)


;//! \htmlinclude resultbag.msg.html

(cl:defclass <resultbag> (roslisp-msg-protocol:ros-message)
  ((img
    :reader img
    :initarg :img
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (grey
    :reader grey
    :initarg :grey
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image))
   (count
    :reader count
    :initarg :count
    :type cl:integer
    :initform 0))
)

(cl:defclass resultbag (<resultbag>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resultbag>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resultbag)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_exercises-msg:<resultbag> is deprecated: use basic_exercises-msg:resultbag instead.")))

(cl:ensure-generic-function 'img-val :lambda-list '(m))
(cl:defmethod img-val ((m <resultbag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_exercises-msg:img-val is deprecated.  Use basic_exercises-msg:img instead.")
  (img m))

(cl:ensure-generic-function 'grey-val :lambda-list '(m))
(cl:defmethod grey-val ((m <resultbag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_exercises-msg:grey-val is deprecated.  Use basic_exercises-msg:grey instead.")
  (grey m))

(cl:ensure-generic-function 'count-val :lambda-list '(m))
(cl:defmethod count-val ((m <resultbag>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_exercises-msg:count-val is deprecated.  Use basic_exercises-msg:count instead.")
  (count m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resultbag>) ostream)
  "Serializes a message object of type '<resultbag>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'grey) ostream)
  (cl:let* ((signed (cl:slot-value msg 'count)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resultbag>) istream)
  "Deserializes a message object of type '<resultbag>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'grey) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'count) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resultbag>)))
  "Returns string type for a message object of type '<resultbag>"
  "basic_exercises/resultbag")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resultbag)))
  "Returns string type for a message object of type 'resultbag"
  "basic_exercises/resultbag")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resultbag>)))
  "Returns md5sum for a message object of type '<resultbag>"
  "d42fed4840bb5096f6e9282c6219991b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resultbag)))
  "Returns md5sum for a message object of type 'resultbag"
  "d42fed4840bb5096f6e9282c6219991b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resultbag>)))
  "Returns full string definition for message of type '<resultbag>"
  (cl:format cl:nil "sensor_msgs/Image img~%sensor_msgs/Image grey~%int32 count~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resultbag)))
  "Returns full string definition for message of type 'resultbag"
  (cl:format cl:nil "sensor_msgs/Image img~%sensor_msgs/Image grey~%int32 count~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resultbag>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'grey))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resultbag>))
  "Converts a ROS message object to a list"
  (cl:list 'resultbag
    (cl:cons ':img (img msg))
    (cl:cons ':grey (grey msg))
    (cl:cons ':count (count msg))
))
