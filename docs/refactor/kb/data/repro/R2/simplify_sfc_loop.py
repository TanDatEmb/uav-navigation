# Control-flow transliteration of polytope.h:199-222 (SimplifySFC greedy merge).
# rep(a,b) = transitionRepresentable(CrossWith(a,b)); inside_tail(k) = sfcs[k] contains tail.
def simplify(n, rep, inside_tail, max_steps=10000):
    check, last = 0, 1
    final=[0]; i=2; steps=0
    while i < n:
        steps+=1
        if steps>max_steps: return final, "NO TERMINATION (steps>%d, final size %d)"%(max_steps,len(final))
        if rep(check,i):
            last=i
            if inside_tail(last): final.append(last); break
        else:
            final.append(last); check=last; i-=1
        i+=1
    return final,"terminated"
# Case A: all adjacent cells representable (normal) -> terminates
print(simplify(5, lambda a,b: abs(a-b)==1, lambda k:k==4))
# Case B: cells 1 and 2 touch (zero-depth / non-enumerable overlap), everything else normal
print(simplify(5, lambda a,b: abs(a-b)==1 and {a,b}!={1,2}, lambda k:k==4)[1])
