(define (domain chip-gt)
    (:requirements :typing :negative-preconditions)
    (:types
        robot location trap animal ranger - object
    )
    (:predicates
        (clear ?l - location)
        (trap-at ?t - trap ?l - location)
        (robot-at ?r - robot ?l - location)
        (ranger-at ?r - ranger ?l - location)
        (adjacent ?l1 ?l2 - location)
        (traversable ?l1 ?l2 - location)
        (spot ?r - robot)
        (drone ?r - robot)
        (trap-push ?t - trap)
        (trap-pic ?t - trap)
        (trap-animal ?t - trap)
        (animal-at ?a - animal ?l - location)
        (robot-carrying-animal ?r - robot ?a - animal)
        (ranger-carrying-animal ?r - ranger ?a - animal)
    )

    (:action move_ground
        :parameters (?from ?to - location ?r - robot)
        :precondition (and
            (robot-at ?r ?from)
            (not (robot-at ?r ?to))
            (adjacent ?from ?to)
            (traversable ?from ?to)
            (spot ?r)
        )
        :effect (and
            (robot-at ?r ?to)
            (not (robot-at ?r ?from))
        )
    )

    (:action move_air
        :parameters (?from ?to - location ?r - robot)
        :precondition (and
            (robot-at ?r ?from)
            (not (robot-at ?r ?to))
            (drone ?r)
        )
        :effect (and
            (robot-at ?r ?to)
            (not (robot-at ?r ?from))
        )
    )

    (:action move_ranger
        :parameters (?from ?to - location ?r - ranger)
        :precondition (and
            (ranger-at ?r ?from)
            (not (ranger-at ?r ?to))
            (adjacent ?from ?to)
        )
        :effect (and
            (ranger-at ?r ?to)
            (not (ranger-at ?r ?from))
        )
    )

    (:action robot_disarm_trap_push
        :parameters (?l - location ?r - robot ?t - trap)
        :precondition (and
            (robot-at ?r ?l)
            (trap-at ?t ?l)
            (trap-push ?t)
            (spot ?r)
        )
        :effect (and
            (clear ?l)
            (not (trap-at ?t ?l))
        )
    )

    (:action ranger_disarm_trap_push
        :parameters (?l - location ?r - ranger ?t - trap)
        :precondition (and
            (ranger-at ?r ?l)
            (trap-at ?t ?l)
            (trap-push ?t)
        )
        :effect (and
            (clear ?l)
            (not (trap-at ?t ?l))
        )
    )

    (:action disarm_trap_pic
        :parameters (?l - location ?r - robot ?t - trap)
        :precondition (and
            (robot-at ?r ?l)
            (trap-at ?t ?l)
            (trap-pic ?t)
        )
        :effect (and
            (clear ?l)
            (not (trap-at ?t ?l))
        )
    )

    (:action disarm_trap_animal
        :parameters (?l - location ?r - robot ?t - trap)
        :precondition (and
            (robot-at ?r ?l)
            (trap-at ?t ?l)
            (trap-animal ?t)
        )
        :effect (and
            (clear ?l)
            (not (trap-at ?t ?l))
        )
    )

    (:action robot_carry_animal
        :parameters (?l - location ?r - robot ?a - animal)
        :precondition (and
            (robot-at ?r ?l)
            (animal-at ?a ?l)
            (clear ?l)
            (spot ?r)
        )
        :effect (and
            (not (animal-at ?a ?l))
            (robot-carrying-animal ?r ?a)
        )
    )

    (:action robot_deliver_animal
        :parameters (?l - location ?r - robot ?a - animal)
        :precondition (and
            (robot-at ?r ?l)
            (robot-carrying-animal ?r ?a)
            (spot ?r)
        )
        :effect (and
            (not (robot-carrying-animal ?r ?a))
            (animal-at ?a ?l)
        )
    )

    (:action ranger_carry_animal
        :parameters (?l - location ?r - ranger ?a - animal)
        :precondition (and
            (ranger-at ?r ?l)
            (animal-at ?a ?l)
            (clear ?l)
        )
        :effect (and
            (not (animal-at ?a ?l))
            (ranger-carrying-animal ?r ?a)
        )
    )

    (:action ranger_deliver_animal
        :parameters (?l - location ?r - ranger ?a - animal)
        :precondition (and
            (ranger-at ?r ?l)
            (ranger-carrying-animal ?r ?a)
        )
        :effect (and
            (not (ranger-carrying-animal ?r ?a))
            (animal-at ?a ?l)
        )
    )
)