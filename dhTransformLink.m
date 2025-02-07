function H = dhTransformLink(link)
    H = dhTransform(link.a, link.d, link.alpha, link.theta);
end