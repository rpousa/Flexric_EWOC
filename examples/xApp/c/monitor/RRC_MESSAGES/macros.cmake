# Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
# contributor license agreements.  See the NOTICE file distributed with
# this work for additional information regarding copyright ownership.
# The OpenAirInterface Software Alliance licenses this file to You under
# the OAI Public License, Version 1.1  (the "License"); you may not use this file
# except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.openairinterface.org/?page_id=698
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#-------------------------------------------------------------------------------
# For more information about the OpenAirInterface (OAI) Software Alliance:
#      contact@openairinterface.org
#

# this function should produce the same value as the macro MAKE_VERSION defined in the C code (file types.h)
function(make_version VERSION_VALUE)
  math(EXPR RESULT "0")
  foreach (ARG ${ARGN})
    math(EXPR RESULT "${RESULT} * 256 + ${ARG}")
  endforeach()
  set(${VERSION_VALUE} "${RESULT}" PARENT_SCOPE)
endfunction()

function(run_asn1c ASN1C_GRAMMAR ASN1C_PREFIX)
  set(options "")
  set(oneValueArgs COMMENT)
  set(multiValueArgs OUTPUT OPTIONS)
  cmake_parse_arguments(ASN1C "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})

  get_filename_component(GRAMMAR_FILE ${ASN1C_GRAMMAR} NAME)
  set(LOGFILE "${CMAKE_CURRENT_BINARY_DIR}/${GRAMMAR_FILE}.log")
  add_custom_command(OUTPUT ${ASN1C_OUTPUT}
    COMMAND ASN1C_PREFIX=${ASN1C_PREFIX} ${ASN1C_EXEC} ${ASN1C_OPTIONS} -D ${CMAKE_CURRENT_BINARY_DIR} ${ASN1C_GRAMMAR} > ${LOGFILE} 2>&1 || cat ${LOGFILE}
    DEPENDS ${ASN1C_GRAMMAR}
    COMMENT "Generating ${ASN1C_COMMENT} from ${GRAMMAR_FILE}"
  )
endfunction()
