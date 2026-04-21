(define (domain chip_gt_problem_enhsp_1-domain)
 (:requirements :strips :typing :negative-preconditions :numeric-fluents)
 (:types location trap robot ranger animal)
 (:predicates (clear ?l - location) (trap_at ?t - trap ?l - location) (robot_at ?r - robot ?l - location) (ranger_at ?r_0 - ranger ?l - location) (adjacent ?l1 - location ?l2 - location) (traversable ?l1 - location ?l2 - location) (spot ?r - robot) (drone ?r - robot) (trap_push ?t - trap) (trap_pic ?t - trap) (trap_animal ?t - trap) (animal_at ?a - animal ?l - location) (robot_carrying_animal ?r - robot ?a - animal) (ranger_carrying_animal ?r_0 - ranger ?a - animal) (inspected ?l - location) (unknown_loc ?l - location))
 (:functions (robot_cost ?r - robot) (ranger_cost ?ran - ranger))
 (:action drone_inspect
  :parameters ( ?pos - location ?r - robot)
  :precondition (and (robot_at ?r ?pos) (drone ?r) (unknown_loc ?pos))
  :effect (and (not (unknown_loc ?pos)) (inspected ?pos) (increase (robot_cost ?r) 1)))
 (:action spot_inspect
  :parameters ( ?pos - location ?r - robot)
  :precondition (and (robot_at ?r ?pos) (spot ?r) (unknown_loc ?pos))
  :effect (and (not (unknown_loc ?pos)) (inspected ?pos) (increase (robot_cost ?r) 1)))
 (:action ranger_inspect
  :parameters ( ?pos - location ?ran - ranger)
  :precondition (and (ranger_at ?ran ?pos) (unknown_loc ?pos))
  :effect (and (not (unknown_loc ?pos)) (inspected ?pos) (increase (ranger_cost ?ran) 1)))
 (:action move_ground
  :parameters ( ?from_ - location ?to - location ?r - robot)
  :precondition (and (robot_at ?r ?from_) (not (robot_at ?r ?to)) (adjacent ?from_ ?to) (traversable ?from_ ?to) (spot ?r))
  :effect (and (robot_at ?r ?to) (not (robot_at ?r ?from_)) (increase (robot_cost ?r) 1)))
 (:action move_air
  :parameters ( ?from_ - location ?to - location ?r - robot)
  :precondition (and (robot_at ?r ?from_) (not (robot_at ?r ?to)) (drone ?r))
  :effect (and (robot_at ?r ?to) (not (robot_at ?r ?from_)) (increase (robot_cost ?r) 1)))
 (:action move_ranger
  :parameters ( ?from_ - location ?to - location ?ran - ranger)
  :precondition (and (ranger_at ?ran ?from_) (not (ranger_at ?ran ?to)) (adjacent ?from_ ?to))
  :effect (and (ranger_at ?ran ?to) (not (ranger_at ?ran ?from_)) (increase (ranger_cost ?ran) 1)))
 (:action robot_disarm_trap_push
  :parameters ( ?l - location ?r - robot ?t - trap)
  :precondition (and (robot_at ?r ?l) (trap_at ?t ?l) (trap_push ?t) (spot ?r))
  :effect (and (clear ?l) (not (trap_at ?t ?l)) (increase (robot_cost ?r) 1)))
 (:action ranger_disarm_trap_pic
  :parameters ( ?l - location ?ran - ranger ?t - trap)
  :precondition (and (ranger_at ?ran ?l) (trap_at ?t ?l) (trap_pic ?t))
  :effect (and (clear ?l) (not (trap_at ?t ?l)) (increase (ranger_cost ?ran) 1)))
 (:action ranger_disarm_trap_animal
  :parameters ( ?l - location ?ran - ranger ?t - trap)
  :precondition (and (ranger_at ?ran ?l) (trap_at ?t ?l) (trap_animal ?t))
  :effect (and (clear ?l) (not (trap_at ?t ?l)) (increase (ranger_cost ?ran) 1)))
 (:action robot_carry_animal
  :parameters ( ?l - location ?r - robot ?a - animal)
  :precondition (and (robot_at ?r ?l) (animal_at ?a ?l) (clear ?l) (spot ?r))
  :effect (and (not (animal_at ?a ?l)) (robot_carrying_animal ?r ?a) (increase (robot_cost ?r) 1)))
 (:action robot_deliver_animal
  :parameters ( ?l - location ?r - robot ?a - animal)
  :precondition (and (robot_at ?r ?l) (robot_carrying_animal ?r ?a) (spot ?r))
  :effect (and (not (robot_carrying_animal ?r ?a)) (animal_at ?a ?l) (increase (robot_cost ?r) 1)))
 (:action ranger_carry_animal
  :parameters ( ?l - location ?ran - ranger ?a - animal)
  :precondition (and (ranger_at ?ran ?l) (animal_at ?a ?l) (clear ?l))
  :effect (and (not (animal_at ?a ?l)) (ranger_carrying_animal ?ran ?a) (increase (ranger_cost ?ran) 1)))
 (:action ranger_deliver_animal
  :parameters ( ?l - location ?ran - ranger ?a - animal)
  :precondition (and (ranger_at ?ran ?l) (ranger_carrying_animal ?ran ?a))
  :effect (and (not (ranger_carrying_animal ?ran ?a)) (animal_at ?a ?l) (increase (ranger_cost ?ran) 1)))
)
