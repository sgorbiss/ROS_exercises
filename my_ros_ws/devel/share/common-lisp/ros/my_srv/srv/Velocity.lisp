; Auto-generated. Do not edit!


(cl:in-package my_srv-srv)


;//! \htmlinclude Velocity-request.msg.html

(cl:defclass <Velocity-request> (roslisp-msg-protocol:ros-message)
  ((min
    :reader min
    :initarg :min
    :type cl:float
    :initform 0.0)
   (max
    :reader max
    :initarg :max
    :type cl:float
    :initform 0.0))
)

(cl:defclass Velocity-request (<Velocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Velocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Velocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_srv-srv:<Velocity-request> is deprecated: use my_srv-srv:Velocity-request instead.")))

(cl:ensure-generic-function 'min-val :lambda-list '(m))
(cl:defmethod min-val ((m <Velocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_srv-srv:min-val is deprecated.  Use my_srv-srv:min instead.")
  (min m))

(cl:ensure-generic-function 'max-val :lambda-list '(m))
(cl:defmethod max-val ((m <Velocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_srv-srv:max-val is deprecated.  Use my_srv-srv:max instead.")
  (max m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Velocity-request>) ostream)
  "Serializes a message object of type '<Velocity-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'min))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'max))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Velocity-request>) istream)
  "Deserializes a message object of type '<Velocity-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'min) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'max) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Velocity-request>)))
  "Returns string type for a service object of type '<Velocity-request>"
  "my_srv/VelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Velocity-request)))
  "Returns string type for a service object of type 'Velocity-request"
  "my_srv/VelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Velocity-request>)))
  "Returns md5sum for a message object of type '<Velocity-request>"
  "9a5456fb608aa5d493111238fa8d046f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Velocity-request)))
  "Returns md5sum for a message object of type 'Velocity-request"
  "9a5456fb608aa5d493111238fa8d046f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Velocity-request>)))
  "Returns full string definition for message of type '<Velocity-request>"
  (cl:format cl:nil "float32 min~%float32 max~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Velocity-request)))
  "Returns full string definition for message of type 'Velocity-request"
  (cl:format cl:nil "float32 min~%float32 max~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Velocity-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Velocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Velocity-request
    (cl:cons ':min (min msg))
    (cl:cons ':max (max msg))
))
;//! \htmlinclude Velocity-response.msg.html

(cl:defclass <Velocity-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (z
    :reader z
    :initarg :z
    :type cl:float
    :initform 0.0))
)

(cl:defclass Velocity-response (<Velocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Velocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Velocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name my_srv-srv:<Velocity-response> is deprecated: use my_srv-srv:Velocity-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Velocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_srv-srv:x-val is deprecated.  Use my_srv-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <Velocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader my_srv-srv:z-val is deprecated.  Use my_srv-srv:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Velocity-response>) ostream)
  "Serializes a message object of type '<Velocity-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Velocity-response>) istream)
  "Deserializes a message object of type '<Velocity-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'z) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Velocity-response>)))
  "Returns string type for a service object of type '<Velocity-response>"
  "my_srv/VelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Velocity-response)))
  "Returns string type for a service object of type 'Velocity-response"
  "my_srv/VelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Velocity-response>)))
  "Returns md5sum for a message object of type '<Velocity-response>"
  "9a5456fb608aa5d493111238fa8d046f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Velocity-response)))
  "Returns md5sum for a message object of type 'Velocity-response"
  "9a5456fb608aa5d493111238fa8d046f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Velocity-response>)))
  "Returns full string definition for message of type '<Velocity-response>"
  (cl:format cl:nil "float32 x~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Velocity-response)))
  "Returns full string definition for message of type 'Velocity-response"
  (cl:format cl:nil "float32 x~%float32 z~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Velocity-response>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Velocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Velocity-response
    (cl:cons ':x (x msg))
    (cl:cons ':z (z msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Velocity)))
  'Velocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Velocity)))
  'Velocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Velocity)))
  "Returns string type for a service object of type '<Velocity>"
  "my_srv/Velocity")