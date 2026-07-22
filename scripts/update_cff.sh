sed -i "s/^version: .*/version: $NEW_VERSION/" CITATION.cff
sed -i "s/^date-released: .*/date-released: $(date "+%Y-%m-%d")/" CITATION.cff