#!/bin/bash
#
# Generate a Markdown-formatted changelog from merge commits.
# Adapted from https://github.com/openlayers/ol3/blob/master/tasks/changelog.sh
#

set -o errexit

#
# Regex to match the standard pull request commit message. Creates capture
# groups for pull request number, commit message body, and merge commit hash.
#
MERGE_RE=Merge\ pull\ request\ #\([0-9]+\)\ from\ [^/]+\/[^\ ]+\ \(.*\)\ \(.*\)

#
# Regex to match the verbose repo remotes output. Creates capture groups for
# repo owner and repo name.
#
REMOTES_RE=.com\/\(.*\)\/\(.*\).git

GITHUB_URL=https://github.com

display_usage() {
  cat <<-EOF

  Usage: ${1} <revision range>|auto

  Creates a Markdown-formatted changelog given a revision range.

  E.g.
      ${1} v1.2.0.. > changelog/v1.2.0.md
      ${1} auto > changelog/latest.md

  See git-log(1) for details on the revision range syntax.

EOF
}

#
# Scan the git log for merge commit messages and output Markdown.  This only
# follows the first parent of merge commits to avoid merges within a topic
# branch (instead only showing merges to master).
#
main() {
  remotes=`git remote -v | sed -n '1!p'`
  if [[ ${remotes} =~ ${REMOTES_RE} ]] ; then
    repo_owner="${BASH_REMATCH[1]}"
    repo_name="${BASH_REMATCH[2]}"
  fi
  PULLS_URL=${GITHUB_URL}/${repo_owner}/${repo_name}/pull

  git log --first-parent --format='%s %b %H' ${1} |
  {
    while read l; do
      if [[ ${l} =~ ${MERGE_RE} ]] ; then
        number="${BASH_REMATCH[1]}"
        summary="${BASH_REMATCH[2]}"
        sha="${BASH_REMATCH[3]}"
        authors=`git log --format='%H' ${sha}^..${sha} | sed -n '1!p' | python commit_authors.py --repo_owner=${repo_owner} --repo_name=${repo_name}`
        echo " * [#${number}](${PULLS_URL}/${number}) - ${summary} (${authors})"
      fi
    done
  }
}

if test ${#} -ne 1; then
  display_usage ${0}
  exit 1
elif [[ ${1} -eq 'auto' ]]; then
    # Prints the changelog between the latest tag and the second last tag
    # tags=(`git log --tags --simplify-by-decoration --pretty="format:%ai %d" -n 2 | sort  | awk -F '[(tag: )]' '{print $(NF-1)}' | xargs`)
    tags=(`git for-each-ref --format="%(refname:short)" --sort=authordate refs/tags | sort | tail -n 2 | xargs`)
    main ${tags[0]}..${tags[1]}
else
  main ${1}
fi
