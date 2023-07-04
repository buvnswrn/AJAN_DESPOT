export JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64
export DESPOT = /home/bhuvanesh/CLionProjects/despot/include/despot
export MY_JNI_HEADERS=${OUT_DIR}/jniHeaders
export PROJECT_PACKAGE=${CURDIR}/java/com/jni/example
export AJAN_PACKAGE=${CURDIR}/java/de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP
export CPP_FOLDER=${CURDIR}/cpp
export MVN_DEPENDENCIES=/home/bhuvanesh/IdeaProjects/pomdp/target
export MVN_SNAPSHOT=${MVN_DEPENDENCIES}/pomdp-1.0-SNAPSHOT.jar
export MVN_DEPENDENCY_FOLDER=${MVN_DEPENDENCIES}/dependency
export DEPENDENCY_FOLDER=/home/bhuvanesh/IdeaProjects/pomdp/target/dependency
export COMMONS_JAR=${DEPENDENCY_FOLDER}/httpclient-4.5.13.jar:${DEPENDENCY_FOLDER}/httpcore-4.4.14.jar
export JSON_JAR=/home/bhuvanesh/IdeaProjects/pomdp/target/dependency/json-20230227.jar
export OTHER_JARS=${DEPENDENCY_FOLDER}/bsh-2.0b4.jar:${DEPENDENCY_FOLDER}/commons-codec-1.11.jar:${DEPENDENCY_FOLDER}/commons-io-2.11.0.jar:${DEPENDENCY_FOLDER}/commons-logging-1.2.jar:${DEPENDENCY_FOLDER}/hamcrest-core-1.3.jar:${DEPENDENCY_FOLDER}/jcommander-1.48.jar:${DEPENDENCY_FOLDER}/testng-6.9.10.jar

export OUT_DIR=${CURDIR}/out
TEMPERATURE_JAVA_FILES := $(wildcard $(PROJECT_PACKAGE)/Temperature*.java)
AJAN_POMDP_FILES := $(wildcard ${AJAN_PACKAGE}/*.java)
AJAN_POMDP_IMPL_FILES := $(wildcard ${AJAN_PACKAGE}/implementation/*.java)
AJAN_UTIL_FILES := $(wildcard ${AJAN_PACKAGE}/util/*.java)
AJAN_PROBLEM_FILES := $(wildcard ${AJAN_PACKAGE}/Problem/*.java)
AJAN_CONNECTOR_FILES := $(wildcard ${AJAN_PACKAGE}/Connector/*.java)
DESPOT_SRC := ${DESPOT}
DESPOT_CORE:= ${DESPOT}/core
DESPOT_INTERFACE:= ${DESPOT}/interface
DESPOT_POMDPX:= ${DESPOT}/pomdpx
DESPOT_SOLVER:= ${DESPOT}/solver
DESPOT_UTIL:= ${DESPOT}/util
DESPOT_UTIL_TINYXML:= ${DESPOT}/util/tinyxml
DESPOT := ${CPP_FOLDER}/despot/build
AJAN_Agent := ${CPP_FOLDER}/AJAN_Agent/src
AJAN_Agent_CPP_Util := ${AJAN_Agent}/jni

#temperaturesampler:
#	mkdir -p ${OUT_DIR} ${MY_JNI_HEADERS}
#	javac -h ${MY_JNI_HEADERS} -d ${OUT_DIR} ${TEMPERATURE_JAVA_FILES}
#	#g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
#	g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TemperatureSampler.cpp -o ${OUT_DIR}/temperatureSampler.o
##	#g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${DESPOT_SRC} -I${DESPOT_CORE} -I${DESPOT_INTERFACE} -I${DESPOT_POMDPX} -I${DESPOT_SOLVER} -I${DESPOT_UTIL} -I${DESPOT_UTIL_TINYXML} -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
##	#g++ -shared -fPIC -o ${OUT_DIR}/libtemperaturesampler.so ${OUT_DIR}/temperatureSampler.o ${OUT_DIR}/tigerState.o -lc
#	g++ -shared -fPIC  ${OUT_DIR}/temperatureSampler.o -o ${OUT_DIR}/libtemperaturesampler.so -lc

AJANPlanner:
	#mvn dependency:copy-dependencies
	mkdir -p ${OUT_DIR} ${MY_JNI_HEADERS} ${AJAN}
	javac -cp ${COMMONS_JAR}:${JSON_JAR}:${OTHER_JARS} -h ${MY_JNI_HEADERS} -d ${OUT_DIR} ${AJAN_POMDP_FILES} ${AJAN_UTIL_FILES} ${AJAN_PROBLEM_FILES} ${AJAN_CONNECTOR_FILES} ${AJAN_POMDP_IMPL_FILES}
	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} \
        ${AJAN_Agent}/ajanAgent.cpp \
        ${AJAN_Agent}/ajanPlanner.cpp \
        ${AJAN_Agent}/AJANBelief.cpp \
        ${AJAN_Agent}/AJANParticleUpperBound.cpp \
        ${AJAN_Agent}/AJANPOMCPPrior.cpp \
        ${AJAN_Agent}/AjanPolicy.cpp \
        ${AJAN_Agent}/ajanWorld.cpp \
        ${AJAN_Agent_CPP_Util}/ajanUtilHelper.cpp \
        ${AJAN_Agent}/jni/ajanHelpers.cpp

#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/jni/ajanHelpers.cpp -o ${OUT_DIR}/ajanHelpers.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/ajanAgent.cpp -o ${OUT_DIR}/ajanAgent.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/ajanPlanner.cpp -o ${OUT_DIR}/ajanPlanner.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/AJANBelief.cpp -o ${OUT_DIR}/ajanBelief.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/AJANParticleUpperBound.cpp -o ${OUT_DIR}/AJANParticleUpperBound.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/AJANPOMCPPrior.cpp -o ${OUT_DIR}/AJANPOMCPPrior.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/AjanPolicy.cpp -o ${OUT_DIR}/AjanPolicy.o
#	g++ -std=c++17 -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${MY_JNI_HEADERS} -I${DESPOT} ${AJAN_Agent}/ajanWorld.cpp -o ${OUT_DIR}/ajanWorld.o
#	#g++ -c -fPIC -I${JAVA_HOME}/include -I${JAVA_HOME}/include/linux -I${DESPOT_SRC} -I${DESPOT_CORE} -I${DESPOT_INTERFACE} -I${DESPOT_POMDPX} -I${DESPOT_SOLVER} -I${DESPOT_UTIL} -I${DESPOT_UTIL_TINYXML} -I${MY_JNI_HEADERS} ${CPP_FOLDER}/TigerState.cpp -o ${OUT_DIR}/tigerState.o
#	#g++ -shared -fPIC -o ${OUT_DIR}/libtemperaturesampler.so ${OUT_DIR}/temperatureSampler.o ${OUT_DIR}/tigerState.o -lc
#	#g++ -std=c++17 -shared -fPIC  ${DESPOT}/*.o ${OUT_DIR}/ajanAgent.o  ${OUT_DIR}/ajanPlanner.o ${OUT_DIR}/ajanBelief.o ${OUT_DIR}/AJANParticleUpperBound.o ${OUT_DIR}/AJANPOMCPPrior.o ${OUT_DIR}/AjanPolicy.o  ${OUT_DIR}/ajanWorld.o -o ${OUT_DIR}/libajanplanner.so -lc
	g++ -std=c++17 -shared -fPIC  ${DESPOT}/*.o -o ${OUT_DIR}/libajanplanner.so *.o

#run_temperaturesampler: temperaturesampler
#	java -cp ${OUT_DIR} -Djava.library.path=${OUT_DIR} com.jni.example.TemperatureSampler

run_planner: AJANPlanner
	#java -cp ${OUT_DIR} -Djava.library.path=${OUT_DIR} com.ajan.POMDP.AJANPlanner
	java -cp ${OUT_DIR}:${COMMONS_JAR}:${JSON_JAR}:${OTHER_JARS} -Djava.library.path=${OUT_DIR} de.dfki.asr.ajan.pluginsystem.mdpplugin.utils.POMDP.AJANPlanner

clean:
	rm -rf ${OUT_DIR} ${MY_JNI_HEADERS}