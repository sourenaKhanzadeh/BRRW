(define (domain plateau_domain_with_plateau)
  (:requirements :strips)
  (:predicates
    (at ?loc - location)
    (connected ?loc1 - location ?loc2 - location)
  )
  (:action move
    :parameters (?from ?to - location)
    :precondition (and (at ?from) (connected ?from ?to))
    :effect (and (not (at ?from)) (at ?to))
  )
)

