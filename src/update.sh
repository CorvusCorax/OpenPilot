#!/bin/sh

function failure {
	echo "Failed with code $?"
	exit
}


rootDir=/home/openpilot/OpenPilot
#rootDir=/home/openpilot/gitsvnrepos

cd "$rootDir"

# update list
echo "fetching remote updates:"
echo "git svn fetch --all --use-log-author"
if git svn fetch --all --use-log-author |grep -qE "r[0-9]* ="; then
#if git svn fetch --all --use-log-author; then
	#new subversion commits

	remoteBranches="$( git branch -r |grep -v origin )";
	localBranches="$( git branch )";

	echo
	echo "Traversing through all branches for rebase"

	for remoteBranch in $remoteBranches; do
		#checkout
		localBranch="$remoteBranch";
		#if [ "$localBranch" = "trunk" ]; then
		#	localBranch="master";
		#fi;
		echo
		echo "updating remote/$remoteBranch to $localBranch"
		if echo "$localBranches" |grep -q "$localBranch"; then
			# local branch exists - rebase
			echo "checking out $localBranch:"
			echo "git checkout $localBranch"
			git checkout "$localBranch" || failure

			#update
			echo "git svn rebase:"
			git svn rebase || failure
		else
			# local branch doesn#t exist - create
			echo "creating $localBranch:"
			echo "git checkout $remoteBranch -b $localBranch"
			git checkout "$remoteBranch" -b "$localBranch" || failure
		fi

	done

	echo
	echo "Bundling together latest subversion reference data in master"

	echo
	echo "git checkout master"
	git checkout master || failure

	echo
	echo "copying files"
	mkdir -p git/svn/refs
	cp -av .git/svn/refs/* git/svn/refs/
	cp -av .git/svn/.metadata git/svn/
	mkdir -p git/refs/remotes

	echo
	echo "creating svn branch references"
	for tag in $( git for-each-ref refs/remotes --format="%(refname):%(objectname)" ); do
		name=${tag/:*}
		sha=${tag/*:}
		if [ "${name%origin*}" = "$name" ]; then
			echo "$sha > git/$name"
			echo $sha >git/$name
		fi
	done

	echo
	echo "adding"
	echo "git add git"
	git add git || failure

	echo
	echo "committing"
	echo "git commit -a -m $( date ) updated latest subversion reference data..."
	git commit -a -m "$( date ) updated latest subversion reference data..." || failure

	echo
	echo "git push origin --all"
	git push origin --all || failure
else
	echo "No new svn commits, nothing to do!"
fi

echo
echo "done."
