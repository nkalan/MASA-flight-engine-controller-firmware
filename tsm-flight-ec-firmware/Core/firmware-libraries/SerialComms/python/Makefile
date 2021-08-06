PYTHON = python3

# Global Defines C files
GLOBALS_H_FILE = ../../../Inc/globals.h
GLOBALS_C_FILE = ../../../Src/globals.c

# GUI Telem Data Templates
TELEM_PARSE_FILE = ${GUI_PATH}/parsers/telemParse.py
GSE_TELEM_PARSE_FILE = ${GUI_PATH}/parsers/gseControllerParse.py
PRESS_TELEM_PARSE_FILE = ${GUI_PATH}/parsers/pressBoardParse.py
FLIGHT_TELEM_PARSE_FILE = ${GUI_PATH}/parsers/flightCompParse.py
EC_TELEM_PARSE_FILE = ${GUI_PATH}/parsers/engineControllerParse.py

EC_CAL_PARSER_FILE = ${GUI_PATH}/parsers/engineControllerCalibrationParse.py

# Telem Data CSV files
TELEM_CSV_FILE = ./telem_data_template.csv
GSE_TELEM_CSV_FILE = ./telem_data_gsecontroller.csv
PRESS_TELEM_CSV_FILE = ./telem_data_pressboard.csv
FLIGHT_TELEM_CSV_FILE= ./telem_data_flightcomp.csv
EC_TELEM_CSV_FILE= ./telem_data_flightec.csv

# Calibration Files
EC_CAL_CSV_FILE = ./calibration_data_flightec.csv
CAL_CSV_FILE	= ./calibrations.csv
CAL_H_FILE	= ../../../Inc/calibrations.h 
CAL_C_FILE	= ../../../Src/calibrations.c

# Telem Command CSV Files
TELEM_CMD_FILE	= ./telem_cmd_template.csv
TELEM_CMD_FILE_DIRECTORY = ${GUI_PATH}
TELEM_C_FILE_DIRECTORY = ../../../Src/

# Board Number Assignments
GSE_BOARD_NUM		= 0
FLIGHT_BOARD_NUM	= 1
EC_BOARD_NUM		= 2
PRESS_BOARD_NUM		= 3

# Note: You need to configure the GUI_PATH variable
# 		in your shell's ~/.bash_profile or ~/.bashrc
# 		in order for the build script to work. You 
# 		should configure it to be the path to your 
# 		MASA gui/python directory

press:
	${PYTHON} ./telem_file_generator.py \
	${PRESS_TELEM_CSV_FILE} ${PRESS_TELEM_PARSE_FILE} \
	${GLOBALS_H_FILE} ${GLOBALS_C_FILE} ${PRESS_BOARD_NUM}
	
	${PYTHON} ./cmd_file_generator.py \
		${TELEM_CMD_FILE} \
		${TELEM_CMD_FILE_DIRECTORY}

	${PYTHON} ./cmd_template_parser.py \
		-f ${TELEM_CMD_FILE} \
		-n ${PRESS_BOARD_NUM} \
		-o ${TELEM_C_FILE_DIRECTORY}

flight:
	${PYTHON} ./telem_file_generator.py \
		${FLIGHT_TELEM_CSV_FILE} ${FLIGHT_TELEM_PARSE_FILE} \
		${GLOBALS_H_FILE} ${GLOBALS_C_FILE} ${FLIGHT_BOARD_NUM}

	${PYTHON} ./cmd_file_generator.py \
		${TELEM_CMD_FILE} \
		${TELEM_CMD_FILE_DIRECTORY}

	${PYTHON} ./cmd_template_parser.py \
		-f ${TELEM_CMD_FILE} \
		-n ${FLIGHT_BOARD_NUM} \
		-o ${TELEM_C_FILE_DIRECTORY}

gse:
	${PYTHON} ./telem_file_generator.py \
		${GSE_TELEM_CSV_FILE} ${GSE_TELEM_PARSE_FILE} \
		${GLOBALS_H_FILE} ${GLOBALS_C_FILE} ${GSE_BOARD_NUM}

	${PYTHON} ./cmd_file_generator.py \
		${TELEM_CMD_FILE} \
		${TELEM_CMD_FILE_DIRECTORY}

	${PYTHON} ./cmd_template_parser.py \
		-f ${TELEM_CMD_FILE} \
		-n ${GSE_BOARD_NUM} \
		-o ${TELEM_C_FILE_DIRECTORY}

ec:
	${PYTHON} ./telem_file_generator.py \
		${EC_TELEM_CSV_FILE} ${EC_TELEM_PARSE_FILE} \
		${GLOBALS_H_FILE} ${GLOBALS_C_FILE} ${EC_BOARD_NUM}

	${PYTHON} ./cmd_file_generator.py \
		${TELEM_CMD_FILE} \
		${TELEM_CMD_FILE_DIRECTORY}

	${PYTHON} ./cmd_template_parser.py \
		-f ${TELEM_CMD_FILE} \
		-n ${EC_BOARD_NUM} \
		-o ${TELEM_C_FILE_DIRECTORY}

gsecal:
	${PYTHON} ./calibration_generator.py	\
		${CAL_CSV_FILE}	\
		${GSE_BOARD_NUM}
		
presscal:
	${PYTHON} ./calibration_generator.py	\
		${CAL_CSV_FILE}	\
		${PRESS_BOARD_NUM}

eccal:
	${PYTHON} ./calibration_file_generator.py	\
		${EC_CAL_CSV_FILE}	\
		${EC_CAL_PARSER_FILE}	\
		${EC_BOARD_NUM}

constants:
	${PYTHON} ./flash_constants_autogen.py	\
		flash_constants.csv \
		0
