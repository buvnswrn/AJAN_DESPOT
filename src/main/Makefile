export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
export DESPOT = /home/bhuvanesh/CLionProjects/despot/include/despot
export MY_JNI_HEADERS=${OUT_DIR}/jniHeaders
export PROJECT_PACKAGE=${PWD}/java/com/jni/example
export AJAN_PACKAGE=${PWD}/java/com/ajan/POMDP
export CPP_FOLDER=${PWD}/cpp

export OUT_DIR=${PWD}/out
TEMPERATURE_JAVA_FILES := $(wildcard $(PROJECT_PACKAGE)/Temperature*.java)
AJAN_POMDP_FILES := $(wildcard ${AJAN_PACKAGE}/*.java)
AJAN_POMDP_IMPL_FILES := $(wildcard ${AJAN_PACKAGE}/implementation/*.java)
DESPOT_SRC := ${DESPOT}
DESPOT_CORE:= ${DESPOT}/core
DESPOT_INTERFACE:= ${DESPOT}/interface
DESPOT_POMDPX:= ${DESPOT}/pomdpx
DESPOT_SOLVER:= ${DESPOT}/solver
DESPOT_UTIL:= ${DESPOT}/util
DESPOT_UTIL_TINYXML:= ${DESPOT}/util/tinyxml
DESPOT := ${CPP_FOLDER}/despot/build

temperaturesampler:
	mkdir -p ${OUT_DIR} ${MY_JNI_HEADERS}
	javac -h ${MY_JNI_HEADERS} -d ${OUT_DIR} ${TEMPERATURE_JAVA_FILES} ${AJAN_POMDP_FILES} ${AJAN_POMDP_IMPL_FILES}
	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TemperatureSampler.cpp -o ${OUT_DIR}/temperatureSampler.o
	#g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${DESPOT_SRC} -I${DESPOT_CORE} -I${DESPOT_INTERFACE} -I${DESPOT_POMDPX} -I${DESPOT_SOLVER} -I${DESPOT_UTIL} -I${DESPOT_UTIL_TINYXML} -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
	#g++ -shared -fPIC -o ${OUT_DIR}/libtemperaturesampler.so ${OUT_DIR}/temperatureSampler.o ${OUT_DIR}/tigerState.o -lc
	g++ -shared -fPIC  ${DESPOT}/*.o ${OUT_DIR}/tigerState.o  ${OUT_DIR}/temperatureSampler.o -o ${OUT_DIR}/libtemperaturesampler.so -lc

run_temperaturesampler: temperaturesampler
	java -cp ${OUT_DIR} -Djava.library.path=${OUT_DIR} com.jni.example.TemperatureSampler

clean:
	rm -rf ${OUT_DIR} ${MY_JNI_HEADERS}