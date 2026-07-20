@matrix @testbot
Feature: Urban Share Matrix
# urban_shares[i][j] is the share of the fastest path through urban-classified
# roads, weighted by the profile's urban_share_weights (testbot: urban = 1.0,
# suburban = 0.5). Cells without a share (diagonal, unreachable pairs,
# fallback_speed estimates) are null. The annotation requires CH and a dataset
# preprocessed with urban_share_weights.

    Background:
        Given the profile "testbot"
        And the partition extra arguments "--small-component-size 1 --max-cell-sizes 2,4,8,16"

    @with_ch
    Scenario: Testbot - Urban share along a mixed urban/suburban/rural line
        Given the node map
            """
            a b c d
            """

        And the ways
            | nodes | urban    |
            | ab    | yes      |
            | bc    | suburban |
            | cd    |          |

        When I request an urban share matrix I should get
            |   | a    | b    | c    | d    |
            | a |      | 1    | 0.75 | 0.5  |
            | b | 1    |      | 0.5  | 0.25 |
            | c | 0.75 | 0.5  |      | 0    |
            | d | 0.5  | 0.25 | 0    |      |

    @with_ch
    Scenario: Testbot - Urban share is null for unreachable pairs
        Given the node map
            """
            a b

            x y
            """

        And the ways
            | nodes | urban |
            | ab    | yes   |
            | xy    |       |

        When I request an urban share matrix I should get
            |   | a | b | x | y |
            | a |   | 1 |   |   |
            | b | 1 |   |   |   |
            | x |   |   |   | 0 |
            | y |   |   | 0 |   |

    @with_ch
    Scenario: Testbot - Urban share stays null for fallback_speed estimated cells
        Given the query options
            | fallback_speed | 5 |

        Given the node map
            """
            a b

            x y
            """

        And the ways
            | nodes | urban |
            | ab    | yes   |
            | xy    |       |

        When I request an urban share matrix I should get
            |   | a | b | x | y |
            | a |   | 1 |   |   |
            | b | 1 |   |   |   |
            | x |   |   |   | 0 |
            | y |   |   | 0 |   |

    @with_ch
    Scenario: Testbot - Urban share requires a dataset preprocessed with urban class weights
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

        When I request an urban share matrix with these waypoints I should get the response code
            | waypoints | code        |
            | a,b       | NoUrbanData |

    @with_mld
    Scenario: Testbot - Urban share is not implemented for MLD
        Given the node map
            """
            a b
            """

        And the ways
            | nodes | urban |
            | ab    | yes   |

        When I request an urban share matrix with these waypoints I should get the response code
            | waypoints | code           |
            | a,b       | NotImplemented |
