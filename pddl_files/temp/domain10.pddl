(define (domain chip_gt-domain)
 (:requirements :strips :typing :negative-preconditions)
 (:types location trap robot ranger animal)
 (:predicates (clear ?l - location) (trap_at ?t - trap ?l - location) (robot_at ?r - robot ?l - location) (ranger_at ?r_0 - ranger ?l - location) (adjacent ?l1 - location ?l2 - location) (traversable ?l1 - location ?l2 - location) (spot ?r - robot) (drone ?r - robot) (trap_push ?t - trap) (trap_pic ?t - trap) (trap_animal ?t - trap) (animal_at ?a - animal ?l - location) (robot_carrying_animal ?r - robot ?a - animal) (ranger_carrying_animal ?r_0 - ranger ?a - animal))
 (:action move_ground
  :parameters ( ?from_ - location ?to - location ?r - robot)
  :precondition (and (robot_at ?r ?from_) (not (robot_at ?r ?to)) (adjacent ?from_ ?to) (traversable ?from_ ?to) (spot ?r))
  :effect (and (robot_at ?r ?to) (not (robot_at ?r ?from_))))
 (:action move_air
  :parameters ( ?from_ - location ?to - location ?r - robot)
  :precondition (and (robot_at ?r ?from_) (not (robot_at ?r ?to)) (drone ?r))
  :effect (and (robot_at ?r ?to) (not (robot_at ?r ?from_))))
 (:action move_ranger
  :parameters ( ?from_ - location ?to - location ?r_0 - ranger)
  :precondition (and (ranger_at ?r_0 ?from_) (not (ranger_at ?r_0 ?to)) (adjacent ?from_ ?to))
  :effect (and (ranger_at ?r_0 ?to) (not (ranger_at ?r_0 ?from_))))
 (:action robot_disarm_trap_push
  :parameters ( ?t - trap ?l - location ?r - robot)
  :precondition (and (robot_at ?r ?l) (trap_at ?t ?l) (trap_push ?t) (spot ?r))
  :effect (and (clear ?l) (not (trap_at ?t ?l))))
 (:action ranger_disarm_trap_push
  :parameters ( ?t - trap ?l - location ?r_0 - ranger)
  :precondition (and (ranger_at ?r_0 ?l) (trap_at ?t ?l) (trap_push ?t))
  :effect (and (clear ?l) (not (trap_at ?t ?l))))
 (:action ranger_disarm_trap_pic
  :parameters ( ?t - trap ?l - location ?r - robot)
  :precondition (and (robot_at ?r ?l) (trap_at ?t ?l) (trap_pic ?t))
  :effect (and (clear ?l) (not (trap_at ?t ?l))))
 (:action disarm_trap_animal
  :parameters ( ?t - trap ?l - location ?r - robot)
  :precondition (and (robot_at ?r ?l) (trap_at ?t ?l) (trap_animal ?t))
  :effect (and (clear ?l) (not (trap_at ?t ?l))))
 (:action robot_carry_animal
  :parameters ( ?l - location ?r - robot ?a - animal)
  :precondition (and (robot_at ?r ?l) (animal_at ?a ?l) (clear ?l) (spot ?r))
  :effect (and (not (animal_at ?a ?l)) (robot_carrying_animal ?r ?a)))
 (:action robot_deliver_animal
  :parameters ( ?l - location ?r - robot ?a - animal)
  :precondition (and (robot_at ?r ?l) (robot_carrying_animal ?r ?a) (spot ?r))
  :effect (and (not (robot_carrying_animal ?r ?a)) (animal_at ?a ?l)))
 (:action ranger_carry_animal
  :parameters ( ?l - location ?r_0 - ranger ?a - animal)
  :precondition (and (ranger_at ?r_0 ?l) (animal_at ?a ?l) (clear ?l))
  :effect (and (not (animal_at ?a ?l)) (ranger_carrying_animal ?r_0 ?a)))
 (:action ranger_deliver_animal
  :parameters ( ?l - location ?r_0 - ranger ?a - animal)
  :precondition (and (ranger_at ?r_0 ?l) (ranger_carrying_animal ?r_0 ?a))
  :effect (and (not (ranger_carrying_animal ?r_0 ?a)) (animal_at ?a ?l)))
)
