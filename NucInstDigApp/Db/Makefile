TOP=../..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this
DB += NucInstDig.db NucInstDigGlobal.db
DB += NucInstDigDCSpec.db NucInstDigTrace.db NucInstDigTOFSpec.db
DB += NucInstDigIntegerParam.db NucInstDigIntegerParamChan.db
DB += NucInstDigRealParam.db NucInstDigRealParamChan.db
DB += NucInstDigStringParam.db NucInstDigStringParamChan.db
DB += ADNucInstDig.template sync_hifi.db

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

