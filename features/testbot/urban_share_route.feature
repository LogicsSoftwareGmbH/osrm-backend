@routing @testbot
Feature: Urban Share Route Annotation
# annotations=urban_share adds a per-segment urban_share array to the leg
# annotation — the class ratio of the road each segment lies on, weighted by
# the profile's urban_share_weights (testbot: urban = 1.0, suburban = 0.5) —
# plus an urban_share summary value per leg. Unlike the table annotation it
# works with both CH and MLD, so these scenarios run under both algorithms.

    Background:
        Given the profile "testbot"
        And the partition extra arguments "--small-component-size 1 --max-cell-sizes 2,4,8,16"

    Scenario: Testbot - Urban share along a mixed urban/suburban/rural line
        Given the node map
            """
            a b c d
            """

        And the ways
            | nodes | urban    | name |
            | ab    | yes      | road |
            | bc    | suburban | road |
            | cd    |          | road |

        And the query options
            | annotations | urban_share |

        When I route I should get
            | from | to | route     | a:urban_share |
            | a    | d  | road,road | 1:0.5:0       |
            | d    | a  | road,road | 0:0.5:1       |

    Scenario: Testbot - Urban share route annotation requires urban class weights
        Given the profile file "testbot" initialized with
        """
        profile.urban_share_weights = nil
        """

        Given the node map
            """
            a b
            """

        And the ways
            | nodes | urban |
            | ab    | yes   |

        And the query options
            | annotations | urban_share |

        When I route I should get
            | from | to | code        |
            | a    | b  | NoUrbanData |
