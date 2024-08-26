(define (domain plateau_domain)
  (:requirements :strips :typing)
  (:types location)
  (:predicates
    (at ?loc - location)
    (connected ?from ?to - location)
    (goal ?loc - location)
  )

  (:action move
    :parameters (?from ?to - location)
    :precondition (and (at ?from) (connected ?from ?to))
    :effect (and (not (at ?from)) (at ?to))
  )
)

