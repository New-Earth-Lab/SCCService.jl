#!/bin/bash
export SUB_CONTROL_URI='aeron:ipc?term-length=512m'
export SUB_CONTROL_STREAM=501
export PUB_STATUS_URI='aeron:ipc?term-length=512m'
export PUB_STATUS_STREAM=502
export SUB_DATA_URI_1='aeron:ipc'
export SUB_DATA_STREAM_1=5003
export PUB_DATA_URI='6000:ipc'
export PUB_DATA_STREAM=2001

export JULIA_PROJECT=/opt/spiders/sccservice/
export JULIA_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=16
export JULIA_LIKWID_PIN="S0:13"

julia +1.10 -e "using SCCService; SCCService.main(ARGS)"