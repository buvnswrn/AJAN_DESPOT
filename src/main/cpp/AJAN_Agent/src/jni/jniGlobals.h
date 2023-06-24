//
// Created by bhuvanesh on 24.06.23.
//
#include <string>

using namespace std;

#ifndef POMDP_JNIGLOBALS_H
#define POMDP_JNIGLOBALS_H

const string STRING = "java/lang/String";
const string VECTOR = "java/util/Vector";
const string OBJECT = "ava/lang/Object";

const string PACKAGE_NAME = "de/dfki/asr/ajan/pluginsystem/mdpplugin/utils/POMDP/";
const string IMPL_PKG = PACKAGE_NAME + "/implementation";

const string STATE = PACKAGE_NAME+ "State";
const string BELIEF = PACKAGE_NAME + "Belief";

const string AJAN_AGENT = IMPL_PKG+ "/AJAN_Agent";
const string AJAN_AGENT_STATE = IMPL_PKG+ "/AJAN_Agent_State";
const string ValuedAction = IMPL_PKG+ "ValuedAction";


string getSig(const string& method) {
    return ("L"+method+";");
}

#endif //POMDP_JNIGLOBALS_H