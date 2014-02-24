;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-sot-year2)

(defgeneric to-msg (data))

(defmethod to-msg ((data xgraph-motion))
  (make-msg
   "robohow_common_msgs-msg/ConstraintConfig"
   :constraints (coerce (mapcar #'to-msg (constraints data)) 'vector)))

(defmethod to-msg ((data xgraph-constraint))
  (let ((param-msg
          (make-msg 
           "robohow_common_msgs-msg/ConstraintCommand"
           :pos_lo (lower data)
           :pos_hi (upper data)
           :selec (selector data)
           :gain (gain data))))
    (make-msg
     "robohow_common_msgs-msg/Constraint"
     :name (id data)
     :function (get-relation-command-code (relation data))
     :tool_feature (to-msg (tool data))
     :world_feature (to-msg (world data))
     :command param-msg)))

(defmethod to-msg ((data xgraph-feature))
  (make-msg
   "robohow_common_msgs-msg/Feature"
   :name (id data)
   :frame_id (frame-id data)
   :type (get-feature-command-code (feature-type data))
   :position (to-msg (origin data))
   :direction (to-msg (orientation data))))

(defmethod to-msg ((data cl-transforms:3d-vector))
  (make-msg 
   "geometry_msgs/Vector3"
   :x (cl-transforms:x data)
   :y (cl-transforms:y data)
   :z (cl-transforms:z data)))

(defun get-relation-command-code (relation-symbol)
  "Returns the XGraph enum defined in robohow_common_msgs/Constraint which corresponds to
 the constraint relation symbol `relation-symbol'."
  (declare (type symbol relation-symbol))
  (roslisp-msg-protocol:symbol-code 'robohow_common_msgs-msg:constraint relation-symbol))

(defun get-feature-command-code (feature-symbol)
  "Returns the XGraph enum defined in robohow_common_msgs/Feature which corresponds
 to the feature symbol `feature-symbol'."
  (declare (type symbol feature-symbol))
  (roslisp-msg-protocol:symbol-code 'robohow_common_msgs-msg:feature feature-symbol))