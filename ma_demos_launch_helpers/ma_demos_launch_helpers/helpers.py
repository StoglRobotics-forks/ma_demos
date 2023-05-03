def delete_direct_slash_duplicate(t):
    if not (t[0] == "/" and t[1] == "/"):
        return t[0]


def prepend_slash_if_not_null(prefix):
    if not prefix:
        return ""
    ns = "/" + prefix
    # remove all occurrences of slashes that directly follow each other ("//Prefix/////Namespace//" -> "/Prefix/Namespace/")
    return "".join(
        filter(
            lambda item: item is not None,
            map(delete_direct_slash_duplicate, zip(ns, ns[1:] + " ")),
        )
    )
