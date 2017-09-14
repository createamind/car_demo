#!/usr/bin/env bash

WORKSPACE_DIR=/tmp/workspace/src
mkdir -p ${WORKSPACE_DIR}/../backup
for DIR in api car_demo prius_description prius_msgs;
do
    [ -L ${WORKSPACE_DIR}/${DIR} ] && continue;
    [ -d ${WORKSPACE_DIR}/${DIR} ] && mv ${WORKSPACE_DIR}/${DIR} ${WORKSPACE_DIR}/../backup
    ln -sf /home/${DIR} ${WORKSPACE_DIR}/${DIR}
done

cd ${WORKSPACE_DIR}
export HISTFILE=${HOME}/.bash_history
touch ${HISTFILE}
export HISTFILESIZE=10000
export HISTSIZE=5000

source /opt/ros/kinetic/setup.bash && source /tmp/workspace/devel/setup.bash
exec /bin/bash -i
