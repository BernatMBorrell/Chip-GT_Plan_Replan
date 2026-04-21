(define (domain chip-gt)
 (:requirements :strips :typing :negative-preconditions :numeric-fluents)
 
 (:types location trap robot)

 (:predicates 
    (r-at ?r - robot ?l - location)
    (t-at ?t - trap ?l - location)
    (safe ?l - location)
    (unreachable ?l - location)
    (r-uav ?r - robot)
    (r-ugv ?r - robot)
    (t-air ?t - trap)
    (t-ground ?t - trap)
    (t-combi ?t - trap)
    (detected ?t - trap)
 )

 (:functions 
    (robot_cost ?r - robot)
 )

 (:action move_uav
  :parameters ( ?r - robot ?from - location ?to - location )
  :precondition (and (r-uav ?r) (r-at ?r ?from) (not (r-at ?r ?to)))
  :effect (and (not (r-at ?r ?from)) (r-at ?r ?to) (increase (robot_cost ?r) 1))
 )

 (:action move_ugv
  :parameters ( ?r - robot ?from - location ?to - location )
  :precondition (and (r-ugv ?r) (r-at ?r ?from) (not (r-at ?r ?to)) (not (unreachable ?to)))
  :effect (and (not (r-at ?r ?from)) (r-at ?r ?to) (increase (robot_cost ?r) 1))
 )

 (:action detect_trap
  :parameters ( ?r - robot ?l - location ?t - trap )
  :precondition (and (r-uav ?r) (r-at ?r ?l) (t-at ?t ?l) (not (detected ?t)))
  :effect (and (detected ?t) (increase (robot_cost ?r) 1))
 )

 (:action disarm_trap_air
  :parameters ( ?r - robot ?l - location ?t - trap )
  :precondition (and (r-uav ?r) (r-at ?r ?l) (t-at ?t ?l) (t-air ?t) (detected ?t))
  :effect (and (safe ?l) (not (t-at ?t ?l)) (increase (robot_cost ?r) 1))
 )

 (:action disarm_trap_ground
  :parameters ( ?r - robot ?l - location ?t - trap )
  :precondition (and (r-ugv ?r) (r-at ?r ?l) (t-at ?t ?l) (t-ground ?t) (detected ?t))
  :effect (and (safe ?l) (not (t-at ?t ?l)) (increase (robot_cost ?r) 1))
 )

 (:action disarm_trap_combi
  :parameters ( ?r1 - robot ?r2 - robot ?l - location ?t - trap )
  :precondition (and (r-uav ?r1) (r-ugv ?r2) (r-at ?r1 ?l) (r-at ?r2 ?l) (t-at ?t ?l) (t-combi ?t) (detected ?t))
  :effect (and (safe ?l) (not (t-at ?t ?l)) (increase (robot_cost ?r1) 1) (increase (robot_cost ?r2) 1))
 )
)
