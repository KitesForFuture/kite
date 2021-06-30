//
// Created by Leonard Koll on 30.06.21.
//

#ifndef KITE_STATEMACHINE_H
#define KITE_STATEMACHINE_H

#include <array>

using namespace std;

template<typename T, int size>
class StateMachine {

    array<T, size> states;
    array<int, size> default_transitions; // Indices into states
    int active_index {0};

public:

    StateMachine(array<T, size> states, array<int, size> default_transitions);
    void next();
    T get_active();
    void set_active(int index);

};

#endif //KITE_STATEMACHINE_H
