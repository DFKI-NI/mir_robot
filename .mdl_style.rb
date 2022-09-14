# Style file for markdownlint

all

exclude_rule 'fenced-code-language' # Fenced code blocks should have a language specified
exclude_rule 'first-line-h1' # First line in file should be a top level header

exclude_rule 'ul-style'

exclude_rule 'no-multiple-blanks'

exclude_rule 'header-style'

exclude_rule 'no-bare-urls'  # we need to use a bare URL in the README for the video to properly show on GitHub

# Line lenght
rule 'MD013', :line_length => 120, :code_blocks => false, :tables => false

# Unordered list indentation
rule 'MD007', :indent => 2

# Ordered list item prefix
rule 'MD029', :style => 'ordered'

rule 'no-duplicate-header', :allow_different_nesting => true
