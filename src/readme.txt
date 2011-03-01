

Make an auto-updating git repository:


step 1:

create new git empty repository with link to subversion

	mkdir OpenPilot
	cd OpenPilot
	git svn init -s svn://svn.openpilot.org/OpenPilot

step 2:
setup master:
copy the files README, LICENSE.txt, link_to_subversion.sh and git/config.add
into place

step 3:
commit that
	git add README LICENSE.TXT git
	git commit -m "initial creation"

step 4:
link to "origin"

	git remote add origin url://to/git/repository
	git push origin --all

step 5:
execute update.sh
(first time will take forever since entire svn history is pulled)


	./update.sh

step 6:
put update.sh into a cronjob and/or commit hook on subversion


step7:

... (possibly setup commit hooks on the upstream repository to automatically
git svn dcommit?)


step 8:
profit

