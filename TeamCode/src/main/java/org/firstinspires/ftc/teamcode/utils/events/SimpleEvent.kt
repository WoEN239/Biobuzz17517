package org.firstinspires.ftc.teamcode.utils.events

import java.util.LinkedList

class SimpleEvent<T> {
    private val listeners = LinkedList<(T) -> Unit>()

    operator fun plusAssign(listener: (T) -> Unit) {
        listeners.add(listener)
    }

    operator fun minusAssign(listener: (T) -> Unit) {
        listeners.remove(listener)
    }

    operator fun invoke(data: T) {
        for (i in listeners)
            i.invoke(data)
    }

    val listenersCount
        get() = listeners.size
}

class SimpleEmptyEvent {
    private val listeners = LinkedList<() -> Unit>()

    operator fun plusAssign(listener: () -> Unit) {
        listeners.add(listener)
    }

    operator fun minusAssign(listener: () -> Unit) {
        listeners.remove(listener)
    }

    operator fun invoke() {
        for (i in listeners)
            i.invoke()
    }

    val listenersCount
        get() = listeners.size
}