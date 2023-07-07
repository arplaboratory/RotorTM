#!/bin/bash

if [[ -z "${ARPL_ROTOR_TM_DIR}" ]]; then  
    echo "ARPL_ROTOR_TM_DIR not available"
    echo "Please export it in your ~/.bashrc"
    exit 1
else  
    echo "ARPL_ROTOR_TM_DIR is "${ARPL_ROTOR_TM_DIR}""    
fi  

find $ARPL_ROTOR_TM_DIR -type f -name "*.py" -exec autopep8 --global-config "${ROTOR_TM_WS}pyproject.toml" {} \;