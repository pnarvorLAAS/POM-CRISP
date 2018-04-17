lib_LTLIBRARIES = libcrisp_genom_codels.la

libcrisp_genom_codels_la_SOURCES=	crisp_genom_c_types.h
libcrisp_genom_codels_la_SOURCES+=	crisp_genom_codels.cc
libcrisp_genom_codels_la_SOURCES+=	crisp_genom_update_internal_tree_codels.cc

libcrisp_genom_codels_la_CPPFLAGS=	$(requires_CFLAGS)
libcrisp_genom_codels_la_LDFLAGS=	$(requires_LIBS)
libcrisp_genom_codels_la_LDFLAGS+=	-release $(PACKAGE_VERSION)


# idl  mappings
BUILT_SOURCES=	crisp_genom_c_types.h
CLEANFILES=	${BUILT_SOURCES}
crisp_genom_c_types.h: ${top_srcdir}/crisp.gen
	${GENOM3}  mappings \
	  -MD -MF .deps/$@.d -MT $@ --signature -l c $< >$@

-include .deps/crisp_genom_c_types.h.d
