(define (problem plateau_problem_with_plateau)
  (:domain plateau_domain_with_plateau)
  (:objects
    loc1 loc2 loc3 loc4 loc5 loc6 loc7 loc8 loc9 loc10 loc11 loc12 - location
  )
  (:init
    (at loc1)
    (connected loc1 loc2)
    (connected loc2 loc3)
    (connected loc3 loc4)
    (connected loc4 loc5)
    (connected loc5 loc6)
    (connected loc6 loc7)
    (connected loc7 loc8)
    (connected loc8 loc9)
    (connected loc9 loc10)
    (connected loc10 loc11)
    (connected loc11 loc12)

    ; Additional misleading connections creating a plateau scenario
    (connected loc4 loc5)
    (connected loc5 loc6)
    (connected loc4 loc6)
    (connected loc6 loc7)
    (connected loc7 loc8)
    (connected loc8 loc9)
    (connected loc9 loc10)
  )
  (:goal (at loc12))
)

