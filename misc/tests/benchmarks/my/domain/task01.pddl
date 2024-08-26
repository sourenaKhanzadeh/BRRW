(define (problem plateau_problem)
  (:domain plateau_domain)
  (:objects
    loc1 loc2 loc3 loc4 loc5 loc6 loc7 loc8 loc9 - location
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
  )
  (:goal (at loc9))
)

