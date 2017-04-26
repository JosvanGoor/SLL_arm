; Auto-generated. Do not edit!


(cl:in-package basic_exercises-srv)


;//! \htmlinclude rgb2grey-request.msg.html

(cl:defclass <rgb2grey-request> (roslisp-msg-protocol:ros-message)
  ((img
    :reader img
    :initarg :img
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass rgb2grey-request (<rgb2grey-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgb2grey-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgb2grey-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_exercises-srv:<rgb2grey-request> is deprecated: use basic_exercises-srv:rgb2grey-request instead.")))

(cl:ensure-generic-function 'img-val :lambda-list '(m))
(cl:defmethod img-val ((m <rgb2grey-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_exercises-srv:img-val is deprecated.  Use basic_exercises-srv:img instead.")
  (img m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgb2grey-request>) ostream)
  "Serializes a message object of type '<rgb2grey-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'img) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgb2grey-request>) istream)
  "Deserializes a message object of type '<rgb2grey-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'img) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgb2grey-request>)))
  "Returns string type for a service object of type '<rgb2grey-request>"
  "basic_exercises/rgb2greyRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgb2grey-request)))
  "Returns string type for a service object of type 'rgb2grey-request"
  "basic_exercises/rgb2greyRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgb2grey-request>)))
  "Returns md5sum for a message object of type '<rgb2grey-request>"
  "e17564393caa09204395429b782c4ee9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgb2grey-request)))
  "Returns md5sum for a message object of type 'rgb2grey-request"
  "e17564393caa09204395429b782c4ee9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgb2grey-request>)))
  "Returns full string definition for message of type '<rgb2grey-request>"
  (cl:format cl:nil "sensor_msgs/Image img~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgb2grey-request)))
  "Returns full string definition for message of type 'rgb2grey-request"
  (cl:format cl:nil "sensor_msgs/Image img~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgb2grey-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'img))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgb2grey-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rgb2grey-request
    (cl:cons ':img (img msg))
))
;//! \htmlinclude rgb2grey-response.msg.html

(cl:defclass <rgb2grey-response> (roslisp-msg-protocol:ros-message)
  ((res
    :reader res
    :initarg :res
    :type sensor_msgs-msg:Image
    :initform (cl:make-instance 'sensor_msgs-msg:Image)))
)

(cl:defclass rgb2grey-response (<rgb2grey-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rgb2grey-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rgb2grey-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name basic_exercises-srv:<rgb2grey-response> is deprecated: use basic_exercises-srv:rgb2grey-response instead.")))

(cl:ensure-generic-function 'res-val :lambda-list '(m))
(cl:defmethod res-val ((m <rgb2grey-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader basic_exercises-srv:res-val is deprecated.  Use basic_exercises-srv:res instead.")
  (res m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rgb2grey-response>) ostream)
  "Serializes a message object of type '<rgb2grey-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'res) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rgb2grey-response>) istream)
  "Deserializes a message object of type '<rgb2grey-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'res) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rgb2grey-response>)))
  "Returns string type for a service object of type '<rgb2grey-response>"
  "basic_exercises/rgb2greyResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgb2grey-response)))
  "Returns string type for a service object of type 'rgb2grey-response"
  "basic_exercises/rgb2greyResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rgb2grey-response>)))
  "Returns md5sum for a message object of type '<rgb2grey-response>"
  "e17564393caa09204395429b782c4ee9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rgb2grey-response)))
  "Returns md5sum for a message object of type 'rgb2grey-response"
  "e17564393caa09204395429b782c4ee9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rgb2grey-response>)))
  "Returns full string definition for message of type '<rgb2grey-response>"
  (cl:format cl:nil "sensor_msgs/Image res~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rgb2grey-response)))
  "Returns full string definition for message of type 'rgb2grey-response"
  (cl:format cl:nil "sensor_msgs/Image res~%~%~%================================================================================~%MSG: sensor_msgs/Image~%# This message contains an uncompressed image~%# (0, 0) is at top-left corner of image~%#~%~%Header header        # Header timestamp should be acquisition time of image~%                     # Header frame_id should be optical frame of camera~%                     # origin of frame should be optical center of cameara~%                     # +x should point to the right in the image~%                     # +y should point down in the image~%                     # +z should point into to plane of the image~%                     # If the frame_id here and the frame_id of the CameraInfo~%                     # message associated with the image conflict~%                     # the behavior is undefined~%~%uint32 height         # image height, that is, number of rows~%uint32 width          # image width, that is, number of columns~%~%# The legal values for encoding are in file src/image_encodings.cpp~%# If you want to standardize a new string format, join~%# ros-users@lists.sourceforge.net and send an email proposing a new encoding.~%~%string encoding       # Encoding of pixels -- channel meaning, ordering, size~%                      # taken from the list of strings in include/sensor_msgs/image_encodings.h~%~%uint8 is_bigendian    # is this data bigendian?~%uint32 step           # Full row length in bytes~%uint8[] data          # actual matrix data, size is (step * rows)~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rgb2grey-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'res))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rgb2grey-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rgb2grey-response
    (cl:cons ':res (res msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rgb2grey)))
  'rgb2grey-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rgb2grey)))
  'rgb2grey-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rgb2grey)))
  "Returns string type for a service object of type '<rgb2grey>"
  "basic_exercises/rgb2grey")