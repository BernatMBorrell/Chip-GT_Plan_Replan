(define (problem chip_gt_problem_enhsp_2-problem)
 (:domain chip_gt_problem_enhsp_2-domain)
 (:objects
   l_start l_01 l_02 l_03 l_end - location
   t_01 t_02 t_03 - trap
   r_drone r_spot - robot
   caro bapt - ranger
   a_01 - animal
 )
 (:init (drone r_drone) (spot r_spot) (= (robot_cost r_drone) 0) (= (robot_cost r_spot) 0) (= (ranger_cost caro) 0) (= (ranger_cost bapt) 0) (robot_at r_drone l_end) (robot_at r_spot l_end) (ranger_at caro l_end) (ranger_at bapt l_end) (adjacent l_start l_01) (adjacent l_01 l_start) (adjacent l_start l_03) (adjacent l_03 l_start) (adjacent l_start l_02) (adjacent l_02 l_start) (adjacent l_01 l_02) (adjacent l_02 l_01) (adjacent l_02 l_03) (adjacent l_03 l_02) (adjacent l_03 l_end) (adjacent l_end l_03) (traversable l_01 l_start) (traversable l_start l_01) (traversable l_end l_03) (traversable l_03 l_end) (traversable l_01 l_02) (traversable l_02 l_01) (traversable l_03 l_start) (traversable l_start l_03) (traversable l_02 l_03) (traversable l_03 l_02) (traversable l_02 l_start) (traversable l_start l_02) (inspected l_start) (clear l_start) (inspected l_end) (clear l_end) (inspected l_01) (clear l_01) (inspected l_02) (clear l_02) (inspected l_03) (clear l_03) (animal_at a_01 l_start))
 (:goal (and (clear l_01) (clear l_02) (clear l_03) (inspected l_01) (inspected l_02) (inspected l_03) (robot_at r_drone l_end) (robot_at r_spot l_end) (ranger_at caro l_end) (ranger_at bapt l_end) (animal_at a_01 l_start)))
 (:metric minimize (+ (* 2 (robot_cost r_spot)) (+ (robot_cost r_drone) (+ (* 4 (ranger_cost bapt)) (* 6 (ranger_cost caro))))))
)
