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

(defclass xgraph-motion ()
  ((constraints :initarg :constraints :reader constraints :type list))
  (:documentation "Common LISP representation of a single motion XGraph."))

(defclass xgraph-constraint ()
  ((id :initarg :it :reader id :type id)
   (relation :initarg :relation :reader relation :type symbol)
   (tool :initarg :tool :reader tool :type xgraph-feature)
   (world :initarg :world :reader world :type xgraph-feature)
   (lower :initarg :lower :reader lower :type vector)
   (upper :initarg :upper :reader upper :type vector)
   (selector :initarg :selector :reader selector :type string)
   (gain :initarg :gain :reader gain :type vector))
  (:documentation "Common LISP representation of an XGraph motion constraint."))

(defclass xgraph-feature ()
  ((id :initarg :id :reader id :type string)
   (frame-id :initarg :frame-id :reader frame-id :type string)
   (feature-type :initarg :feature-type :reader feature-type :type symbol)
   (origin :initarg :origin :reader origin :type cl-transforms:3d-vector)
   (orientation :initarg :orientation :reader orientation :type cl-transforms:3d-vector))
  (:documentation "Common LISP representation of a 3D-feature for XGraph motion specs."))