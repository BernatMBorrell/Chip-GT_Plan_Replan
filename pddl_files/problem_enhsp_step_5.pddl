(define (problem chip_gt_problem_enhsp_5-problem)
 (:domain chip_gt_problem_enhsp_5-domain)
 (:objects
   l_start l_01 l_02 l_03 l_04 l_05 l_06 l_07 l_end - location
   t_01 t_02 t_03 t_04 t_05 t_06 t_07 - trap
   r_drone r_spot - robot
   caro bapt - ranger
   a_01 - animal
 )
 (:init (drone r_drone) (spot r_spot) (= (robot_cost r_drone) 0) (= (robot_cost r_spot) 0) (= (ranger_cost caro) 0) (= (ranger_cost bapt) 0) (robot_at r_drone l_end) (robot_at r_spot l_end) (ranger_at caro l_end) (ranger_at bapt l_end) (adjacent l_start l_01) (adjacent l_01 l_start) (adjacent l_start l_06) (adjacent l_06 l_start) (adjacent l_01 l_02) (adjacent l_02 l_01) (adjacent l_01 l_end) (adjacent l_end l_01) (adjacent l_01 l_04) (adjacent l_04 l_01) (adjacent l_01 l_07) (adjacent l_07 l_01) (adjacent l_01 l_05) (adjacent l_05 l_01) (adjacent l_02 l_03) (adjacent l_03 l_02) (adjacent l_03 l_04) (adjacent l_04 l_03) (adjacent l_03 l_07) (adjacent l_07 l_03) (adjacent l_03 l_06) (adjacent l_06 l_03) (adjacent l_03 l_end) (adjacent l_end l_03) (adjacent l_04 l_05) (adjacent l_05 l_04) (adjacent l_05 l_06) (adjacent l_06 l_05) (adjacent l_06 l_07) (adjacent l_07 l_06) (adjacent l_07 l_end) (adjacent l_end l_07) (traversable l_end l_01) (traversable l_01 l_end) (traversable l_01 l_02) (traversable l_02 l_01) (traversable l_06 l_07) (traversable l_07 l_06) (traversable l_03 l_04) (traversable l_04 l_03) (traversable l_06 l_05) (traversable l_05 l_06) (traversable l_04 l_01) (traversable l_01 l_04) (traversable l_05 l_04) (traversable l_04 l_05) (traversable l_07 l_01) (traversable l_01 l_07) (traversable l_03 l_02) (traversable l_02 l_03) (traversable l_end l_07) (traversable l_07 l_end) (traversable l_01 l_start) (traversable l_start l_01) (traversable l_end l_03) (traversable l_03 l_end) (inspected l_start) (clear l_start) (inspected l_end) (clear l_end) (inspected l_01) (clear l_01) (inspected l_02) (clear l_02) (inspected l_03) (clear l_03) (inspected l_04) (clear l_04) (inspected l_05) (clear l_05) (inspected l_06) (clear l_06) (inspected l_07) (clear l_07) (animal_at a_01 l_start))
 (:goal (and (clear l_01) (clear l_02) (clear l_03) (clear l_04) (clear l_05) (clear l_06) (clear l_07) (inspected l_01) (inspected l_02) (inspected l_03) (inspected l_04) (inspected l_05) (inspected l_06) (inspected l_07) (robot_at r_drone l_end) (robot_at r_spot l_end) (ranger_at caro l_end) (ranger_at bapt l_end) (animal_at a_01 l_start)))
 (:metric minimize (+ (* 2 (robot_cost r_spot)) (+ (robot_cost r_drone) (+ (* 4 (ranger_cost bapt)) (* 6 (ranger_cost caro))))))
)
