(define (problem chip_gt-problem)
 (:domain chip_gt-domain)
 (:objects
   l_01 l_02 l_03 l_04 l_05 l_06 l_07 l_08 l_09 l_10 l_start l_end - location
   t_01 t_02 t_03 t_04 t_05 t_06 t_07 t_08 t_09 t_10 - trap
   r_drone r_spot - robot
   caro bapt - ranger
   a_01 - animal
 )
 (:init (drone r_drone) (spot r_spot) (robot_at r_drone l_start) (robot_at r_spot l_start) (ranger_at caro l_start) (ranger_at bapt l_start) (adjacent l_start l_01) (adjacent l_01 l_start) (traversable l_start l_01) (traversable l_01 l_start) (adjacent l_02 l_01) (adjacent l_01 l_02) (traversable l_02 l_01) (traversable l_01 l_02) (adjacent l_03 l_02) (adjacent l_02 l_03) (traversable l_03 l_02) (traversable l_02 l_03) (adjacent l_04 l_03) (adjacent l_03 l_04) (traversable l_04 l_03) (traversable l_03 l_04) (adjacent l_05 l_04) (adjacent l_04 l_05) (traversable l_05 l_04) (traversable l_04 l_05) (adjacent l_06 l_05) (adjacent l_05 l_06) (traversable l_06 l_05) (traversable l_05 l_06) (adjacent l_07 l_06) (adjacent l_06 l_07) (traversable l_07 l_06) (traversable l_06 l_07) (adjacent l_08 l_07) (adjacent l_07 l_08) (traversable l_08 l_07) (traversable l_07 l_08) (adjacent l_09 l_08) (adjacent l_08 l_09) (traversable l_09 l_08) (traversable l_08 l_09) (adjacent l_10 l_end) (adjacent l_end l_10) (traversable l_10 l_end) (traversable l_end l_10) (adjacent l_10 l_09) (adjacent l_09 l_10) (traversable l_10 l_09) (traversable l_09 l_10) (clear l_01) (clear l_02) (trap_at t_03 l_03) (trap_animal t_03) (trap_at t_04 l_04) (trap_push t_04) (trap_at t_05 l_05) (trap_animal t_05) (clear l_06) (clear l_07) (clear l_08) (clear l_09) (clear l_10) (animal_at a_01 l_02))
 (:goal (and (clear l_01) (clear l_02) (clear l_03) (clear l_04) (clear l_05) (clear l_06) (clear l_07) (clear l_08) (clear l_09) (clear l_10) (robot_at r_drone l_end) (robot_at r_spot l_end) (ranger_at caro l_end) (ranger_at bapt l_end) (animal_at a_01 l_start)))
)
