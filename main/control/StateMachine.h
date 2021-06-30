//
// Created by Leonard Koll on 30.06.21.
//

#ifndef KITE_STATEMACHINE_H
#define KITE_STATEMACHINE_H

#include <array>

using namespace std;

template<typename T, int size>
class StateMachine {

    array<T*, size> states;
    array<int, size> default_transitions; // Indices into states
    int active_index {0};

public:

    StateMachine(array<T*, size> states, array<int, size> default_transitions);
    void next();
    T* get_active();
    void set_active(int index);

};

template<typename T, int size>
StateMachine<T, size>::StateMachine(array<T*, size> states, array<int, size> default_transitions) :
        states{states}, default_transitions{default_transitions}
{}

template<typename T, int size>
void StateMachine<T, size>::next() {
    active_index = default_transitions[active_index];
}

template<typename T, int size>
T* StateMachine<T, size>::get_active() {
    return states[active_index];
}

template<typename T, int size>
void StateMachine<T, size>::set_active(int index) {
    if (index >= 0 && index < size) {
        active_index = index;
    }
}

#endif //KITE_STATEMACHINE_H
