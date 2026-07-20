@routing @car
Feature: Car - Urban classifier legal-zone value handling
# Locks the classify_zone_value rules of lib/urban_classifier.lua end-to-end
# through the car profile (urban = 1.0, suburban = 0.5, rural = 0), using the
# route annotation as the observable. Covers the forms the DACH taginfo
# research identified as load-bearing: the wiki-standard zone:maxspeed number
# form (DE:30, ~163k ways), Austrian Ortstafel limits (AT:city_limit30), bare
# zone numbers, and the traffic-sign-code guard — DE:sign:274-30 carries a
# number but is an explicit sign, not a zone statement, and must stay rural.

    Background:
        Given the profile "car"
        Given a grid size of 200 meters

    Scenario: Car - Legal-zone value forms classify by their meaning
        Given the node map
            """
            a b c d e f g
            """

        And the ways
            | nodes | highway | name | zone:maxspeed | source:maxspeed | maxspeed |
            | ab    | primary | road | DE:30         |                 |          |
            | bc    | primary | road |               | DE:sign:274-30  |          |
            | cd    | primary | road |               |                 | 50       |
            | de    | primary | road |               | AT:city_limit30 |          |
            | ef    | primary | road | DE:100        |                 |          |
            | fg    | primary | road | 30            |                 |          |

        And the query options
            | annotations | urban_share |

        When I route I should get
            | from | to | route     | a:urban_share |
            | a    | b  | road,road | 1             |
            | b    | c  | road,road | 0             |
            | c    | d  | road,road | 0.5           |
            | d    | e  | road,road | 1             |
            | e    | f  | road,road | 0             |
            | f    | g  | road,road | 1             |

    Scenario: Car - Street lighting is a suburban fallback on major roads too
        Given the node map
            """
            a b c d
            """

        And the ways
            | nodes | highway   | name | lit |
            | ab    | secondary | road | yes |
            | bc    | secondary | road |     |
            | cd    | primary   | road | yes |

        And the query options
            | annotations | urban_share |

        When I route I should get
            | from | to | route     | a:urban_share |
            | a    | b  | road,road | 0.5           |
            | b    | c  | road,road | 0             |
            | c    | d  | road,road | 0.5           |

    Scenario: Car - A lit maxspeed=50 arterial is urban, unlit stays suburban
    # 50 is the DACH built-up default; street lighting corroborates the
    # missing urban context tag. Corroboration stops above 50: a lit 60 is
    # plain suburban via the numeric tier.
        Given the node map
            """
            a b c d e
            """

        And the ways
            | nodes | highway   | name | maxspeed | lit |
            | ab    | secondary | road | 50       | yes |
            | bc    | secondary | road | 50       |     |
            | cd    | secondary | road | 50       | no  |
            | de    | primary   | road | 60       | yes |

        And the query options
            | annotations | urban_share |

        When I route I should get
            | from | to | route     | a:urban_share |
            | a    | b  | road,road | 1             |
            | b    | c  | road,road | 0.5           |
            | c    | d  | road,road | 0.5           |
            | d    | e  | road,road | 0.5           |
