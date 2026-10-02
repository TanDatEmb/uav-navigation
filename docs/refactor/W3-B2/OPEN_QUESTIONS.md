# W3-B2 open questions

1. What pure trajectory/world witness type replaces `CandidateCommandBundle`
   at the certifier boundary without moving backend ownership?
2. Should stop reachability receive a backend-provided bounded synthesis
   callback, or should the synthesis result become a pure data input?
3. Which namespace is authoritative for `candidateMatchesAnchor`: the existing
   execution copy or the duplicate candidate-bundle copy?
4. Which existing tests are the moved certifier oracle, and which tests remain
   backend wrapper tests?
